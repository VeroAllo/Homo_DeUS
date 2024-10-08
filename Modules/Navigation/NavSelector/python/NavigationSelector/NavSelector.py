#!/usr/bin/env python3


from homodeus_library.homodeus_precomp import *
from NavigationSelector.NavGoalDeserializer import NavGoalDeserializer
from NavigationSelector.NavGoal import NavGoal
from homodeus_msgs.msg import *

BEHAVIOR_GOTO_SUBTOPIC = "/Homodeus/Behaviour/Goto"
BEHAVIOR_ROTATE_SUBTOPIC = "/Homodeus/Behaviour/TurnAround"
PERCEPTION_ROBOT_POSE_TOPIC = "/Homodeus/Perception/RobotPose"

class NavSelector :
    __instance = None
    __fileName : String = ""

    def __new__(cls):
        if cls.__instance is None :
            cls.__instance = super(NavSelector, cls).__new__(cls)
        return cls.__instance

    def __init__(self, goalList : List[NavGoal] = [], currentGoal : NavGoal = None, topic : String = BEHAVIOR_GOTO_SUBTOPIC, filename : String = None) -> None:
        self.__currentGoal : NavGoal = currentGoal
        self.__currentLocation : Tuple[Point,Quaternion] = getPose() #remove if not working
        self.__goalList : List[NavGoal] = goalList
        self.__goalSent : NavGoal  = None

        self.client = None
        self.counter = 0
        self.__currentID = 0

        self.__callBack = None
        self.__isActive : bool = False
        self.__callSubBehavior_BaseRotate = False

        self.__hz = rospy.get_param('~hz', 10)
        self.__topic : String = topic
        self.__initConnectionToNode()
        rospy.on_shutdown(self.__closeConnectionToNode)

    def ConnectCallBack(self,callBackFunction) -> None :
        self.__callBack = callBackFunction

    def GetFilename(self) -> str : return self.__filename

    def GetGoalList(self) -> List[NavGoal] : return self.__goalList

    def GetCurrentGoal(self) -> NavGoal : return self.__currentGoal

    def GetState(self) -> GoalStatus : return self.client.get_state()

    def SetFilename(self, filename:str) -> str : self.__filename = filename

    def SetCurrentGoal(self, goal : NavGoal) -> None : self.__currentGoal = goal

    def SetIndexCurrentGoal(self, index_goal : int) -> None :
        nbElem = len(self.GetGoalList()) 
        if nbElem != 0 and index_goal < nbElem :
            self.SetCurrentGoal(self.GetGoalList()[index_goal])

    # Public Functions NavBehavior
    def AddGoalNav(self, goal : NavGoal) -> None :
        # Si but assigne est impossible d'atteindre, annuler le but et informer qui de droit
        if self.ImpossibleGoal(goal):
            print(f"Impossible go to goal ({goal.GetPoint()}) from robot's pose")
            self.__SendResponseToHBBA(self.__currentID, GoalStatus.REJECTED)
        else:
            if self.__currentGoal is None :
                self.__currentGoal = goal
            else :
                self.__goalList.append(goal)
                print(f"Nombre de goal dans la liste : {len(self.__goalList)}")

    def AddGoalPose(self, pose : HDPose) -> None :
        self.__currentID = pose.id.desire_id

        p, q = pose.pose.position, pose.pose.orientation        
        x, y, z = p.x, p.y, p.z
        w = quarternion2euler(q).z

        self.AddGoalNav(NavGoal(x, y, z, w, "GoalPose"))

    def AddGoal(self, goalX : float, goalY : float, goalZ : float, goalOri : float, name : str) -> None :
        nav_goal = NavGoal(goalX, goalY, goalZ, goalOri, name)
        self.AddGoalNav(nav_goal)

    def BlockGoal(self, index : int) -> None :
        nbElem = len(self.GetGoalList()) 
        if nbElem != 0 and index < nbElem :
            self.__goalList[index].BlockNavGoal()
            return
        hdWarn("Navigation Selector - No goal matches this index, no goal will be blocked")
        
    def BlockAllGoals(self) -> None :
        for goal in self.GetGoalList() :
            goal.BlockNavGoal()

    def CancelAllGoals(self) -> None:
        if self.client != None:
            self.client.cancel_all_goals()

    def ExtendGoals(self, goals : List[NavGoal]) -> None :
        self.__goalList.extend(goals)

    def LoadPreDefNavGoal(self) -> None:
        if not hasattr(self, '__navGoalSerializer'):
            self.__navGoalSerializer = NavGoalDeserializer(fileName=self.__filename)
        self.__goalList = []
        self.__navGoalSerializer.Read(self.ExtendGoals)

    def NbUnblockedTask(self) -> int :
        cpt = 0
        for goal in self.GetGoalList() :
            if not goal.IsBlocked() :
                cpt += 1
        return cpt

    def RemoveGoal(self, index : int) -> None :
        del self.__goalList[index]

    def RemoveCurrentGoal(self) -> None : #prob rename
        for goal in self.GetGoalList() :
            if (not goal.IsBlocked()) :
                self.SetCurrentGoal(goal)
                self.__goalList.remove(goal) # prone to cause error, maybe just wait after the loop then delete
                self.__currentLocation = getPose()
                return
        self.SetCurrentGoal(None)
        hdWarn("Navigation Selector - No NavGoal capable to be run at the current time, waiting for more")

    def UnblockGoal(self, index : int) -> None :
        nbElem = len(self.GetGoalList()) 
        if nbElem != 0 and index < nbElem :
            self.GetGoalList()[index].UnblockNavGoal()
            if (self.GetCurrentGoal() is None):
                self.RemoveCurrentGoal()
            return
        hdWarn("Navigation Selector - No goal matches this index, no goal will be unblocked")

    def UnblockAllGoals(self) -> None :
        for goal in self.GetGoalList() :
            goal.UnblockNavGoal()

    # Private Functions Events
    def __OnEvent(self, eventContent) -> None :
        if eventContent is not None and (self.GetCurrentGoal() is not None  or self.__goalSent is not None):
            hdInfo(f"Navigation Selector - Event triggered by the current Navigation Goal (NavGoalID = {self.__goalSent.GetNavGoalID()})")
            self.__callBack(eventContent)

    def __OnNavGoalFail(self, errorDesc : str, goalStatus : GoalStatus) -> None :
        hdErr(f"Navigation Selector - Failed to fulfill the current goal (NavGoalID = {self.__goalSent.GetNavGoalID()}), removed from the list and notified the main controller \
                \nGoal created at {self.__goalSent.GetCreationTime()}, \
                \nTime taken for this resolution = {self.__goalSent.GetNavGoalExistenceTime()} \
                \nGoalStatus = {convGoalStatus(goalStatus)} \
                \nGoalStatus Value = {goalStatus} \
                \nError associated : {errorDesc}")
        self.__OnEvent(goalStatus)

    def __OnNavGoalSuccess(self) -> None :
        hdInfo(f"Navigation Selector - Current goal (NavGoalID = {self.__goalSent.GetNavGoalID()}), succeeded, beginning next NavGoal when the controller is ready \
                    \nGoal created at {self.__goalSent.GetCreationTime()}, \
                    \nTime taken for the success = {self.__goalSent.GetNavGoalExistenceTime()}")
        self.__OnEvent(NAVGOALSUCCESS)

    def HandleNodeTaskEnd(self, endState, _) -> None:
        success = False
        if endState == 0:
            self.__OnNavGoalFail(NAVGOALFAILED,endState)
        elif endState == GoalStatus.SUCCEEDED :
            success = True
            self.__OnNavGoalSuccess()
        else :
            self.__OnNavGoalFail(NAVGOALFAILED,endState)

        self.__SendResponseToHBBA(self.__currentID, success)
        
        self.RemoveCurrentGoal()
        self.__goalSent = None

    # Send Goal To Base
    def SendGoal(self) -> None:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        if self.client.get_state() not in [GoalStatus.PENDING]:
            if self.GetCurrentGoal() is not None and not self.GetCurrentGoal().IsBlocked() : 
                print(self.__currentGoal)
                posPoint, oriQuat = self.GetCurrentGoal().GetPose()
                goal.target_pose.pose.position = posPoint
                goal.target_pose.pose.orientation = oriQuat
                self.client.send_goal(goal=goal,done_cb=self.HandleNodeTaskEnd)
                self.__goalSent = self.__currentGoal
                self.__currentGoal = None
            elif self.GetCurrentGoal() is None and self.NbUnblockedTask() == 0 :
                hdInfo("Navigation Selector - No unblocked task left to select, throwing event to the controller")
                self.__OnEvent(NOUNBLOCKEDNAVGOAL)
            elif self.GetCurrentGoal() is None and len(self.GetGoalList()) > 0 :
                self.RemoveCurrentGoal()
            elif self.GetCurrentGoal() is None and len(self.GetGoalList()) == 0 :
                hdInfo("Navigation Selector - No tasks left to select, throwing event to the controller")
                self.__OnEvent(NONAVGOALREMAINING)

    # Private Functions Topic ROS
    def __SendResponseToHBBA(self, id: int, value: int) -> None:
        response : HDResponse = HDResponse()
        response.id.desire_id = id
        response.result = value
        self.__goto_response_pub.publish(response)

    def __SendStatusToHBBA(self, id: int, value: int) -> None:
        status : HDStatus = HDStatus()
        status.id.desire_id = id
        status.status = value
        self.__goto_status_pub.publish(status)

    def __turn_around_subscriber(self, value: Int8Stamped) -> None:
        self.__callSubBehavior_BaseRotate = True

    def __robot_pose_subscriber(self, robot_pose: RobotPoseStamped) -> None:
        self.__robot_pose = robot_pose.pose

    # def __robot_pose_subscriber(self, poseWCS: PoseWithCovarianceStamped) -> None:
    #     self.__robot_pose = poseWCS.pose.pose
    
    # Private Functions Service ROS
    def __initSrvMoveBase(self) -> None:
        ACTION_NAME = "move_base"
        TIMEOUT_SRV = 5

        try:
            self.client = actionlib.SimpleActionClient(ACTION_NAME, MoveBaseAction)
            if not self.client.wait_for_server(timeout=rospy.Duration(TIMEOUT_SRV)):
                print("MoveBaseAction hit timeout ... isn't available !")
        except rospy.ServiceException as src_exc:
            print(f"Service {ACTION_NAME} ne repond pas pcq {src_exc}")

    def __initSrvGetPlan(self) -> None:
        """
        Service make_plan (/move_base/make_plan) donne le chemin du point A au point B
        (vide si impossible de faire un chemin)

        https://docs.ros.org/en/api/nav_msgs/html/srv/GetPlan.html
        https://docs.ros.org/en/noetic/api/nav_msgs/html/srv/GetPlan.html
        req = nav_msgs.GetPlan()
        https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html
        resp = __srv_get_plan(req.start, req.goal, req.tolerance)
        resp.poses -> Si chemin vide, alors impossible d'atteindre le point B p/r au A
        """
        SRV_NAME_GET_PLAN = '/move_base/make_plan'
        TIMEOUT_SRV = 10
        try:
            rospy.wait_for_service(SRV_NAME_GET_PLAN, timeout=rospy.Duration(TIMEOUT_SRV))
            if not hasattr(self, '__srv_get_plan'):
                self.__srv_get_plan = rospy.ServiceProxy(SRV_NAME_GET_PLAN, GetPlan)
        except rospy.ServiceException as src_exc:
            print(f"Service {SRV_NAME_GET_PLAN} ne repond pas pcq {src_exc}")

    def RelocateItselfInMap(self, call_recovery_rotation=False) -> None:
        SRV_NAME_RELOCATE = '/global_localization'
        TIMEOUT_SRV = 5

        try:
            # arreter le but en cours
            if self.client.get_state() in [GoalStatus.PENDING]:
                self.client.cancel_goal()

            if self.__goalSent is not None:
                self.__goalList.insert(0, self.__goalSent)
                self.__goalSent = None

            # lancer le service du filtre de particules
            rospy.wait_for_service(SRV_NAME_RELOCATE, timeout=rospy.Duration(TIMEOUT_SRV))
            if not hasattr(self, '__srv_relocate'):
                self.__srv_relocate = rospy.ServiceProxy(SRV_NAME_RELOCATE, Empty)
            self.__srv_relocate()

            if call_recovery_rotation:
                self.__callSubBehavior_BaseRotate = False
                # envoyer un but impossible pour avoir le 'rotate_recovery'
                recovery_goal = NavGoal(float('inf'), float('inf'), 0, 0, 'rotate_recovery')
                # self.__goalList.insert(0, recovery_goal)
                self.SetCurrentGoal(recovery_goal)
                # self.SendGoal()
            else:
                self.__callSubBehavior_BaseRotate = True
                # utiliser le comportement implemente par homodeus

                rotate_goal = Float32Stamped()
                rotate_goal.header.stamp = rospy.Time.now()
                rotate_goal.header.frame_id = ''
                rotate_goal.value = 3*pi

                self.__base_rotate_goal_pub.publish(rotate_goal)

        except rospy.ServiceException as src_exc:
            print(f"Service {SRV_NAME_RELOCATE} ne repond pas pcq {src_exc}")

    def ClearMap(self) -> None:
        SRV_NAME_CLEAR = '/move_base/clear_costmaps'
        TIMEOUT_SRV = 5

        try:
            rospy.wait_for_service(SRV_NAME_CLEAR, timeout=rospy.Duration(TIMEOUT_SRV))
            if not hasattr(self, '__srv_clear'):
                self.__srv_clear = rospy.ServiceProxy(SRV_NAME_CLEAR, Empty)
            self.__srv_clear()
        except rospy.ServiceException as src_exc:
            print(f"Service {SRV_NAME_CLEAR} ne repond pas pcq {src_exc}")

    def ImpossibleGoal(self, nav_goal: NavGoal) -> bool:
        pose_start  = PoseStamped()
        pose_start.header.stamp = rospy.Time(0)
        pose_start.header.frame_id = 'map'
        pose_start.pose.position.x = self.__robot_pose.x
        pose_start.pose.position.y = self.__robot_pose.y
        q = quaternion_from_euler(0,0,self.__robot_pose.z)
        pose_start.pose.orientation= Quaternion(q[0],q[1],q[2],q[3])
        # pose_start.pose.position   = self.__robot_pose.position
        # pose_start.pose.orientation= self.__robot_pose.orientation

        pose_end    = PoseStamped()
        pose_end.header.stamp = rospy.Time.now()
        pose_end.header.frame_id = 'map'
        pose_end.pose.position = nav_goal.GetPoint()
        pose_end.pose.orientation = nav_goal.GetOri()

        request_plan = GetPlan()
        request_plan.start = pose_start
        request_plan.goal = pose_end
        # Perimetre autour ou le robot peut aller (zone tampon)
        request_plan.tolerance = 0.0254

        response = self.__srv_get_plan(request_plan.start, request_plan.goal, request_plan.tolerance)
        return len(response.plan.poses) == 0

    # Private Function Node ROS
    def __initConnectionToNode(self) -> None :
        self.__goto_response_pub = rospy.Publisher(self.__topic+"/Response", HDResponse, queue_size = 10,  latch = False)
        self.__goto_status_pub = rospy.Publisher(self.__topic+"/Status", HDStatus, queue_size = 10,  latch = False)
        self.__goto_request_sub = rospy.Subscriber(self.__topic+"/Request", HDPose, callback = self.AddGoalPose, queue_size = 10)

        self.__base_rotate_goal_pub = rospy.Publisher(BEHAVIOR_ROTATE_SUBTOPIC + "/Request", Float32Stamped, queue_size=1)
        self.__turn_around_sub = rospy.Subscriber(BEHAVIOR_ROTATE_SUBTOPIC+"/Response", Int8Stamped, self.__turn_around_subscriber)

        self.__robot_pose_sub = rospy.Subscriber(PERCEPTION_ROBOT_POSE_TOPIC, RobotPoseStamped, self.__robot_pose_subscriber)
        # self.__robot_pose_sub = rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.__robot_pose_subscriber)

        self.__initSrvMoveBase()
        self.__initSrvGetPlan()
        self.ClearMap()

        self.__rate = rospy.Rate(self.__hz)

    def __closeConnectionToNode(self) -> None :
        self.__goalList.clear()
        if self.client is not None and self.client.get_state() == GoalStatus.ACTIVE :
            self.client.cancel_goal()
        self.__currentGoal = None

        if hasattr(self, '__goto_response_pub'): self.__goto_response_pub.unregister()
        if hasattr(self, '__goto_status_pub'): self.__goto_status_pub.unregister()
        if hasattr(self, '__goto_request_sub'): self.__goto_request_sub.unregister()
        if hasattr(self, '__base_rotate_goal_pub'): self.__base_rotate_goal_pub.unregister()
        if hasattr(self, '__turn_around_sub'): self.__turn_around_sub.unregister()
        if hasattr(self, '__robot_pose_sub'): self.__robot_pose_sub.unregister()

    def Sort(self) -> None:
        #self.__goalList.sort() #Need to sort with a key, which key, I don't know
        pass

    def Behave(self):
        if self.counter % 10 == 0 :
            ...#print("currently behaving")

        self.counter += 1

        if not self.__callSubBehavior_BaseRotate:
            if self.__currentGoal is not None and self.__goalSent is None:
                self.Sort()
                self.SendGoal()

            if self.__goalSent is not None:
                self.__SendStatusToHBBA(self.__currentID, self.GetState())

    def __display_menu(self) -> None:
        #system("clear")
        print('(0) Just continue')
        print('(1) Block next task')
        print('(2) Unblock next task')
        print('(3) Block all tasks')
        print('(4) Unblock all tasks')
        print('(5) Reload the predef goals')
        print('(6) Clear Terminal')
        print('(7) Get status of the current goal')
        print('(8) Cancel/Abort current goal')
        print('(9) Quit')
        print('(10) Set new goal')
        print('(11) Dump the goals list in json')
        print('(12) Print the nav goal list')
        if debug :
            print(f'Nombre de Navigation goal ici présent dans le pays du québec \n Nb = {len(self.GetGoalList())}')
            print(f'Nombre de unblocked task = {self.NbUnblockedTask()}')
            if (self.GetCurrentGoal() is not None):
                print(f'Current goal id : {self.GetCurrentGoal().GetNavGoalID()}')

    def run(self) -> None: #Will only manually control the class for now, once the HBBA controller works, will be automatic
        # self.__initConnectionToNode()     # Move to __init__()

        if (not DEBUG_NAV_SELECTOR) :
            print("starting the navSelector in automatic mode")
            while not rospy.is_shutdown() :
                self.Behave()
                self.__rate.sleep()
            return

        running = True
        while running:
            self.__display_menu()
            strInput : str = input("Please enter your selection number: ")
            choice = int(strInput) if strInput.isnumeric() else None
 
            if choice == 0:
                self.SendGoal()
            elif choice == 1:
                self.BlockGoal(0)
            elif choice == 2:
                self.UnblockGoal(0)
            elif choice == 3:
                self.BlockAllGoals()
            elif choice == 4:
                self.UnblockAllGoals()
            elif choice == 5:
                self.__goalList = []
                self.__navGoalSerializer.Read(self.ExtendGoals)
            elif choice == 6:
                system("clear")
            elif choice == 7:
                print(convGoalStatus(self.client.get_state()))
            elif choice == 8:
                self.client.cancel_goal()
                pass
            elif choice == 10:
                goalName : str = input("Please enter your goal name: ")
                strGoal : str = input("Please enter your coor (x y w) without (): ")
                coords = [float(x) for x in strGoal.split()]
                if len(coords) == 3:
                    self.AddGoal(coords[0], coords[1], 0.0, coords[2], goalName)
            elif choice == 11:
                self.__navGoalSerializer.Write(self.__goalList)
            elif choice == 12:
                for goal in self.GetGoalList(): print(goal)
            # Connect to BehavoirNode (turn_around if necessary)
            elif choice == 13:
                self.RelocateItselfInMap()
            elif choice == 14:
                self.ClearMap()
            # Simulate Motivation / Desir
            elif choice == 15:
                goalName : str = input("Please enter your goal name: ")                
                self.AddGoal(coords[0], coords[1], 0.0, coords[2], goalName)
            elif choice == 9:
                print("Bye")
                running = False
            else:
                print("Choice is not valid")

            self.__rate.sleep()
