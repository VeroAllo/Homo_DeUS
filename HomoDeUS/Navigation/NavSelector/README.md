## Sous-module de navigation, comportement « aller à »

Exemple d'utilisation du module en simulation : 

1. Identifier le module sur lequel vous voulez simuler (par exemple pmb2_2dnav_gazebo)
2. Créer un fichier scripts et ajouter le NavSelector
3. Ajouter un MAKEFILE et ajouter comme exécutable le NavSelector
    - Donner les permissions au NavSelector
4. Lancer la simulation
5. Lancer le module en utilisant rosrun

# NavSelector (singleton)

# Class Variables

## Not currently used

- BEHAVIOR_ROTATE_SUBTOPIC : str
- PERCEPTION_ROBOT_POSE_TOPIC : str

- __navGoalSerializer : NavGoalDeserializer

## Used at the moment

### Public

- counter : int

### Private

- __filename : str
- __instance : NavSelector

- __callBack = None (Supposed to be a Function)
- __currentGoal : NavGoal
- __currentID : int
- __currentLocation : Tuple[Point,Quaternion]
- __currentName : str
- __goalList : List[NavGoal]
- __goalSent : NavGoal
- __hz : int
- __isActive : bool
- __rate : Rate
- __topic : String

# Class Methods

## Python Class Method override

```python
__init__(self, goalList : List[NavGoal] = [], currentGoal : NavGoal = None, topic : String = None) -> None
```

### Function Explanation

- Class constructor of the NavigationSelector, can be created with default value or specified values.
- Creates a [NavGoalDeserializer](https://www.notion.so/NavGoalDeserializer-95c472c1c47a488187a42921d07e5cfd?pvs=21) to get Goals to use from the Json in the file.
- Fetches the current position on load from "moveBaseActionFeedback.base_position.pose".
- Returns nothing.

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goalList | List[NavGoal] | [] | List of goal that were created before the class |
| currentGoal | Navgoal | None | First goal that we want to force |
| topic | String | None | The topic that we want to use [UNUSED FOR THE MOMENT] |
| filename  | String | None | The filename of the json |

 

## Class methods

### Public methods

### ConnectCallBack

```python
ConnectCallBack(self,callBackFunction) -> None
```

### Function Explanation

- Connects the NavSelector to a single function passed by the controller
- Returns nothingGetGoalList(self) -> List[NavGoal]

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| callBackFunction | Function [Not enforced as I was not sure how to type it] | No default param | A CallBack function taking any function taking a parameter |

### GetFilename

```python
GetFilename(self) -> str
```

### Function Explanation

- Returns a copy of the filename

### GetGoalList

```python
GetGoalList(self) -> List[NavGoal]
```

### Function Explanation

- Returns a copy of the goalList

### GetCurrentGoal

```python
GetCurrentGoal(self) -> NavGoal
```

### Function Explanation

- Return the goal that will be processed

### SetFilename

```python
SetFilename(self, filename : str) -> str
```

### Function Explanation

- Sets the filename (json)
- Returns a copy of filename

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| filename | str | No default value | The filename of the json |

### SetCurrentGoal

```python
SetCurrentGoal(self, goal : NavGoal) -> None
```

### Function Explanation

- Sets the current goal to another one
- Returns Nothing
- [Maybe should add a boolean param to see if we want to keep the goal that we currently discard]

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goal | NavGoal | No default value | The goal we wish to apply |

### SetIndexCurrentGoal

```python
SetIndexCurrentGoal(self, index_goal : int) -> None
```

### Function Explanation

- Sets the current goal to another specific goal in the goalList
- The specific goal is choice with a valid index
- Returns Nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| index_goal | int | No default value | The index goal we wish to apply |

### AddGoalNav

```python
AddGoalNav(self, goal : NavGoal) -> None
```

### Function Explanation

- Adds a goal to the goal list if it is reachable goal and currentGoal is empty
- No special insertion, so the list is currently unsorted
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goal | NavGoal | No default value | The goal we wish to append to the goal list |

### AddGoalPose

```python
AddGoalPose(self, pose : HDPose) -> None
```

### Function Explanation

- Receive a goal from specific topic
- ReConstructor the goal in format NavGoal
- Pass the goal AddGoalNav function
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| pose | HDPose | No default value | The goal receive of specific topic and we wish to append to the goal list |

### AddGoal

```python
AddGoal(self, goalX : float, goalY : float, goalZ : float, goalOri : float, name : String) -> None
```

### Function Explanation

- Creates a goal to the goalList
- Create the goal using the parameters passed
- - Pass the goal AddGoalNav function
- For more precisions on the constructor, see [NavGoal](https://www.notion.so/NavSelector-cf58eced786747e793e5cd1ef96abaea?pvs=21)

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goalX  | Float | No default value | The X position of the goal |
| goalY | Float | No default value | The Y position of the goal |
| goalZ  | Float | No default value | The Z position of the goal |
| goalOri  | Float | No default value | The orientation of the goal |
| name | str | No default value | The name of the goal |

### ExtendGoals

```python
ExtendGoals(self, goals : List[NavGoal]) -> None
```

### Function Explanation

- Adds a list of goal to the goal list
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goals | List[NavGoal] | No default values | The list of goal we wish to add to the goal list |

### CancelAllGoals

```python
CancelAllGoals(self) -> None
```

### Function Explanation

- Cancel all goal send to actionClient _move_base_
- Returns nothing

### RemoveGoal

```python
RemoveGoal(self, index : int) -> None
```

### Function Explanation

- Remove the goal at the specified index
- Currently, no check if it is possible [Could cause problems]
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| index | int | No default value | Index is currently not checked |

### GetState

```python
GetState(self) -> GoalStatus
```

### Function Explanation

- The values' state are in homodeus_precomp.py
- Returns the state of actionClient _move_base_

### RemoveCurrentGoal

```python
RemoveCurrentGoal(self) -> None
```

### Function Explanation

- Removes the current goal and tries to get an unblocked goal to replace the now vacant current goal
- Checks sequentially if the goals are blocked
    - If one is found, then we : set the current goal, remove the goal in the goal list, sets the current pose (position and angle in `Point` and `Quaternion` respectively) and returns.
    - If none is found, then we : set the current goal to the `None` and we send a warning.
- Returns nothing

### BlockGoal

```python
BlockGoal(self, index : int) -> None
```

### Function Explanation

- Blocks the goal at the specified index, so that we won’t execute it if it unless we unblock it
- Checks if the index is coherent
    - If the index is coherent, then we : block it and return
    - If the index is not coherent, then we send a warning
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| index | int | No default value | Index to block |

### BlockAllGoals

```python
BlockAllGoals(self) -> None
```

### Function Explanation

- Blocks all goal in the list, but won’t affect the current goal
- Returns nothing

### UnblockGoal

```python
UnblockGoal(self, index : int) -> None
```

### Function Explanation

- Unblocks the goal at the specified index
- Checks if the index is coherent
    - If the index is coherent, then we : block it and return
    - If the index is not coherent, then we send a warning
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| index | int  | No default value | Index to unblock |

### UnblockAllGoals

```python
UnblockAllGoals(self) -> None
```

### Function Explanation

- Unblocks all goals in the list, but won’t affect the current goal
- Returns nothing

### NbUnblockedTask

```python
NbUnblockedTask(self) -> int
```

### Function Explanation

- Checks all goals in the goal list to count the number of unblocked task
- Returns the count

### run

```python
run(self) -> None
```

### Function Explanation

- Initalize the connection to node (call `initConnectionToNode`)
- Run in automatic mode (wait goal from HBBA) if `DEBUG_NAV_SELECTOR` is False
- Display menu and calls functions upon selection else
- Returns nothing

### Private Methods

### __OnNavGoalFail

```python
__OnNavGoalFail(self, errorDesc : String, goalStatus : GoalStatus) -> None
```

### Function Explanation

- Function decorator that will throw an error [non-blocking] to the controller using a call to the `__OnEvent(eventContent)`
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| errorDesc | Description of the error | No default value | Description associated to the error |
| goalStatus | GoalStatus | No default value | GoalStatus from rospy, useful to know what failed |

### __OnNavGoalSuccess

```python
__OnNavGoalSuccess(self) -> None
```

### Function Explanation

- Function decorator that will print the success of the goal and send an event to the controller using a call to the `__OnEvent(eventContent)`
- Returns nothing

### __OnEvent

```python
__OnEvent(self, eventContent) -> None
```

### Function Explanation

- Sends an event to the controller using the callBack that is connected, prints that an event was sent
- Won’t do anything if the eventContent is None and if we don’t have a current goal [Should still be the same as this function is called *before* the goal is changed]
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| eventContent | Any | No default value | The content that we want to transmit to the controller |

### Sort

```python
Sort(self) -> None
```

### Function Explanation

- Unimplemented for now
- Should be private function
- Should sort the goal list by key(s)
- Returns nothing

### HandleNodeTaskEnd

```python
HandleNodeTaskEnd(self, endState, _) -> None
```

### Function Explanation

- Call at the task end
- Select the good __OnNavGoal depending on goal status
- Set the head position after navigate
- Send a response to applicant (in this case: HBBA)
- Remove the current goal
- Should be private function
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| endState | Any | No default value | The value of end state |

### SendGoal

```python
__sendGoal(self) -> None
```

### Function Explanation

- Sends the goal to the predefined topic
    - Checks for coherence of the current goal then : updates the goal in the `MoveBaseGoal`, on completion of the goal we send events to the controller whether or not the goal has succeeded
    - If there are no current goal and unblocked task, will warn the controller
    - If there are no current goal and no goal in the goal list, will warn the controller
- Should be private function
- Returns nothing

### __display_menu

```python
__display_menu(self) -> None:
```

### Function Explanation

- Show the debug menu with options to select

### initSrvGetPlan

```python
initSrvGetPlan(self) -> None
```

### Function Explanation

- Initialize the connection with service `make_plan`
- Should be a private function
- Returns nothing

### __defineOrientationHead

```python
__defineOrientationHead(self, name:str="") -> float, float
```

### Function Explanation

- Select pitch and yaw for orientation head
- Selection is make with name's goal receive
- Returns pitch and yaw

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| name | str | Empty string | The goal name receive from sender |

### __controlHead

```python
__controlHead(self, pitch:float=0, yaw:float=0) -> None
```

### Function Explanation

- Construct the head trajectory
- Send the trajectory and wait
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| pitch | float | 0 | Lateral axis [-0.5, 0.0] |
| yaw | float | 0 | Vertical axis [-0.5, 0.5] |

### __SendResponseToHBBA

```python
__SendResponseToHBBA(self, id: int, value: int) -> None
```

### Function Explanation

- Publish response to HBBA at the end execution goal
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| id | int | No default value | Id receive in the resquest goal |
| value | int | No default | End statut from resquet goal |

### __SendStatusToHBBA

```python
__SendStatusToHBBA(self, id: int, value: int) -> None
```

### Function Explanation

- Publish status to HBBA during execution goal
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| id | int | No default value | Id receive in the resquest goal |
| value | int | No default | Current statut from resquet goal |

### initConnectionToNode

```python
initConnectionToNode(self) -> None
```

### Function Explanation

- Initialize the ROS Node (publish, subscrib topic, action_clients)
- Should be a private function
- Returns nothing

### closeConnectionToNode

```python
closeConnectionToNode(self) -> None
```

### Function Explanation

- Clear the goat list
- Cancel the current goals
- Release the ROS resources (publish, subscrib topic, action_clients)
- Should be a private function
- Returns nothing

### RelocateItselfInMap

```python
RelocateItselfInMap(self) -> None
```

### ClearMap

```python
ClearMap(self) -> None
```

### ImpossibleGoal

```python
ImpossibleGoal(self, nav_goal: NavGoal) -> bool
```

### Function Explanation

- Valid the resquet goal is reachable
- Should be a private function
- Returns True if it is not, False it is

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| nav_goal | NavGoal | No default value | The resquest goal to valid |

### IThinkIKnowWhereIAm

```python
IThinkIKnowWhereIAm(self) -> bool
```

### LoadPreDefNavGoal

```python
LoadPreDefNavGoal(self) -> None
```

### Function Explanation

- Load predefined NavGoal from filename json
- Should be a private function
- Returns nothing

### Behave

```python
Behave(self)
```

### Function Explanation

- Call `SendGoal` if currentGoal is not empty and `goalSent` is empty 
- Should be a private function
- Returns nothing
