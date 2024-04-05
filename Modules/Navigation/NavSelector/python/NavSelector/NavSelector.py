#!/usr/bin/env python3


from homodeus_library.homodeus_precomp import *
# from NavGoalDeserializer import NavGoalDeserializer
from NavSelector.NavGoalDeserializer import NavGoalDeserializer
# from NavGoal import NavGoal
from NavSelector.NavGoal import NavGoal
from homodeus_msgs.msg import Int8Stamped, Float32Stamped, RobotPoseStamped


class NavSelector :
    __instance = None
    __fileName : String = ""

    def __new__(cls):
        if cls.__instance is None :
            cls.__instance = super(NavSelector, cls).__new__(cls)
        return cls.__instance

    def __init__(self, goalList : List[NavGoal] = [], currentGoal : NavGoal = None, topic : String = "Homodeus/Behaviour/Goto", filename : String = None) -> None:
        self.__currentGoal : NavGoal = currentGoal
        self.__goalSent : NavGoal  = None
        self.__goalList : List[NavGoal] = goalList
        self.__topic : String = topic
        self.__hz = rospy.get_param('~hz', 10)
        self.__currentLocation : Tuple[Point,Quaternion] = getPose() #remove if not working
        self.__callBack = None
        self.__isActive : bool = False
        #self.__navGoalSerializer = NavGoalDeserializer(filename)
        #self.__navGoalSerializer.Read(self.ExtendGoals)
        self.__rate =rospy.Rate(self.__hz)
        self.counter = 0
        self.__currentID = 0
        rospy.on_shutdown(self.closeConnectionToNode)

    def ConnectCallBack(self,callBackFunction) -> None :
        self.__callBack = callBackFunction

    def GetFilename(self) -> str :
        return self.__filename

    def GetGoalList(self) -> List[NavGoal] :
        return self.__goalList

    def GetCurrentGoal(self) -> NavGoal :
        return self.__currentGoal

    def SetFilename(self, filename:str) -> str :
        self.__filename = filename

    def SetCurrentGoal(self, goal : NavGoal) -> None :
        self.__currentGoal = goal

    def SetIndexCurrentGoal(self, index_goal : int) -> None :
        nbElem = len(self.GetGoalList()) 
        if nbElem != 0 and index_goal < nbElem :
            self.SetCurrentGoal(self.GetGoalList()[index_goal])

    def AddGoalNav(self, goal : NavGoal) -> None :
        # A titre informatif, pourrait-etre utilise comme info a retransmettre a HBBA
        if self.ImpossibleGoal(goal):
            print(f"Impossible go to goal ({goal.GetPoint()}) from robot's pose")

        if self.__currentGoal is None :
            self.__currentGoal = goal
        else :
            self.__goalList.append(goal)

    def AddGoalPose(self, pose : HDPose) -> None :
        #peut-être, amener un status ici

        self.__currentID = pose.id.desire_id
        print("Goal was received sending back a response")
        p, q = pose.pose.position, pose.pose.orientation        
        x, y, z = p.x, p.y, p.z
        w = quarternion2euler(q).z
        self.AddGoalNav(NavGoal(x, y, z, w, "NoName"))
        print(f"Nombre de goal dans la liste : {len(self.__goalList) + 1}")

    def AddGoal(self, goalX : float, goalY : float, goalZ : float, goalOri : float, name : str) -> None :
        nav_goal = NavGoal(goalX, goalY, goalZ, goalOri, name)
        self.AddGoalNav(nav_goal)

    def ExtendGoals(self, goals : List[NavGoal]) -> None :
        self.__goalList.extend(goals)

    def CancelAllGoals(self) -> None:
        if self.client != None:
            self.client.cancel_all_goals()

    def RemoveGoal(self, index : int) -> None :
        del self.__goalList[index]

    def GetState(self) -> GoalStatus :
        return self.client.get_state()

    def RemoveCurrentGoal(self) -> None : #prob rename
        for goal in self.GetGoalList() :
            if (not goal.IsBlocked()) :
                self.SetCurrentGoal(goal)
                self.__goalList.remove(goal) # prone to cause error, maybe just wait after the loop then delete
                self.__currentLocation = getPose()
                return
        self.SetCurrentGoal(None)
        hdWarn("Navigation Selector - No NavGoal capable to be run at the current time, waiting for more")

    def BlockGoal(self, index : int) -> None :
        nbElem = len(self.GetGoalList()) 
        if nbElem != 0 and index < nbElem :
            self.__goalList[index].BlockNavGoal()
            return
        hdWarn("Navigation Selector - No goal matches this index, no goal will be blocked")
        
    def BlockAllGoals(self) -> None :
        for goal in self.GetGoalList() :
            goal.BlockNavGoal()

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

    def NbUnblockedTask(self) -> int :
        cpt = 0
        for goal in self.GetGoalList() :
            if not goal.IsBlocked() :
                cpt += 1
        return cpt

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

    def __OnEvent(self, eventContent) -> None :
        if eventContent is not None and (self.GetCurrentGoal() is not None  or self.__goalSent is not None):
            hdInfo(f"Navigation Selector - Event triggered by the current Navigation Goal (NavGoalID = {self.__goalSent.GetNavGoalID()})")
            self.__callBack(eventContent)

    def Sort(self) -> None:
        #self.__goalList.sort() #Need to sort with a key, which key, I don't know
        pass

    def HandleNodeTaskEnd(self, endState, _) -> None:
        success = False
        if endState == 0:
            self.__OnNavGoalFail(NAVGOALFAILED,endState)
        elif endState == GoalStatus.SUCCEEDED :
            success = True
            self.__OnNavGoalSuccess()
        else :
            self.__OnNavGoalFail(NAVGOALFAILED,endState)
        response : HDResponse = HDResponse()
        response.id.desire_id = self.__currentID
        response.result = success
        self.__publisher.publish(response)
        self.RemoveCurrentGoal()
        self.__goalSent = None

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
                hdInfo("Navigation Selector - No tasks left to select, throwing event to the controller")
                self.__OnEvent(NONAVGOALREMAINING)
            elif self.GetCurrentGoal() is None and len(self.GetGoalList()) == 0 :
                self.RemoveCurrentGoal()

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

    def __robot_pose_subscriber(self, poseWCS: PoseWithCovarianceStamped) -> None:
        self.__robot_pose = poseWCS.pose.pose

    def initSrvGetPlan(self) -> None:
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
        try:
            srv_name_get_plan = '/move_base/make_plan'
            rospy.wait_for_service(srv_name_get_plan, timeout=rospy.Duration(5))
            if not hasattr(self, '__srv_get_plan'):
                self.__srv_get_plan = rospy.ServiceProxy(srv_name_get_plan, GetPlan)
        except rospy.ServiceException as src_exc:
            print(f"Service {srv_name_get_plan} ne repond pas pcq {src_exc}")

    def initConnectionToNode(self) -> None :
        self.__publisher = rospy.Publisher(self.__topic+"/Response", HDResponse, queue_size = 10,  latch = False)
        self.__subscriber = rospy.Subscriber(self.__topic+"/Request", HDPose, callback = self.AddGoalPose, queue_size = 10)
        self.__robot_pose_sub = rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.__robot_pose_subscriber)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            print("MoveBaseAction isn't available !")

        self.initSrvGetPlan()

        self.__rate = rospy.Rate(self.__hz)

    def closeConnectionToNode(self) -> None :
        self.__goalList.clear()
        if self.client.get_state() == GoalStatus.ACTIVE :
            self.client.cancel_goal()
        self.__currentGoal = None
        self.__publisher.unregister()
        self.__subscriber.unregister()
        self.__robot_pose_sub.unregister()

    def RelocateItselfInMap(self) -> None :
        try:
            # arreter le but en cours
            if self.client.get_state() in [GoalStatus.PENDING]:
                self.client.cancel_goal()

            # lancer le service du filtre de particules
            srv_name_relocate = '/global_localization'
            rospy.wait_for_service(srv_name_relocate, timeout=rospy.Duration(5))
            if not hasattr(self, '__srv_relocate'):
                self.__srv_relocate = rospy.ServiceProxy(srv_name_relocate, Empty)
            self.__srv_relocate()

            # envoyer un but impossible pour avoir le 'rotate_recovery'
            recovery_goal = NavGoal(float('inf'), float('inf'), 0, 0, 'rotate_recovery')
            self.__goalList.insert(0, recovery_goal)
            self.SetCurrentGoal(recovery_goal)
            self.SendGoal()
        except rospy.ServiceException as src_exc:
            print(f"Service {srv_name_relocate} ne repond pas pcq {src_exc}")

    def ClearMap(self) -> None:
        try:
            srv_name_clear = '/move_base/clear_costmaps'
            rospy.wait_for_service(srv_name_clear, timeout=rospy.Duration(5))
            if not hasattr(self, '__srv_clear'):
                self.__srv_clear = rospy.ServiceProxy(srv_name_clear, Empty)
            self.__srv_clear()
        except rospy.ServiceException as src_exc:
            print(f"Service {srv_name_clear} ne repond pas pcq {src_exc}")

    def ImpossibleGoal(self, nav_goal: NavGoal) -> bool:
        pose_start  = PoseStamped()
        pose_start.header.stamp = rospy.Time(0)
        pose_start.header.frame_id = 'map'
        pose_start.pose.position   = self.__robot_pose.position
        pose_start.pose.orientation= self.__robot_pose.orientation

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

    def __pose_cb(self, poseWCS) -> None:
        pose: Pose = poseWCS.pose.pose
        covar_mat : float[36] = poseWCS.pose.covariance

        max_value = abs(max(covar_mat, key=abs))
        mat_sum = sum([i for i in covar_mat])
        print(f"Max {max_value}; Sum {mat_sum}")

        self.__IThinkIAmNotLost = (max_value < 1 and mat_sum < 0.05)

    def IThinkIKnowWhereIAm(self) -> bool:
        if not hasattr(self, '__pose_covar_sub'):
            topic_name = '/amcl_pose'
            self.__pose_covar_sub = rospy.Subscriber(topic_name, PoseWithCovarianceStamped, self.__pose_cb)
            self.__IThinkIAmNotLost = False

        return self.__IThinkIAmNotLost
    
    def LoadPreDefNavGoal(self) -> None:
        if not hasattr(self, '__navGoalSerializer'):
            self.__navGoalSerializer = NavGoalDeserializer(fileName=self.__filename)
        self.__goalList = []
        self.__navGoalSerializer.Read(self.ExtendGoals)
        # for goal in self.GetGoalList():
        #     print(goal)

    def Behave(self):
        if self.counter % 10 == 0 :
            ...#print("currently behaving")

        self.counter += 1

        if self.__currentGoal is not None and self.__goalSent is None:
            self.Sort()
            self.SendGoal()

    def run(self) -> None: 
        self.initConnectionToNode()
        if (not DEBUG_NAV_SELECTOR) :
            print("starting the navSelector in automatic mode")
            while not rospy.is_shutdown() :
                self.Behave()
                self.__rate.sleep()
            return

        # if self.GetCurrentGoal() is None and len(self.GetGoalList()) != 0 :
        #     self.SetCurrentGoal(self.GetGoalList()[0])
        #     del self.__goalList[0] #Scuffed
        running = True
        while running:
            self.__display_menu()
            strInput : str = input("Please enter your selection number: ")
            if not strInput.isnumeric():
                choice = None
            else:
                choice = int(strInput)
 
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
                for goal in self.GetGoalList():
                    print(goal)
            elif choice == 9:
                print("Bye")
                running = False
            else:
                print("Choice is not valid")

            self.__rate.sleep()