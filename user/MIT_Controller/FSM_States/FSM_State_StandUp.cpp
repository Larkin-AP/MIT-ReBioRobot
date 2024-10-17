/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandUp.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */

/*
FSM_State_StandUp<T> 是一个模板类，用来处理机器人站立状态。
构造函数 接受一个 ControlFSMData<T>* 参数 _controlFSMData，传递给父类 FSM_State<T> 进行初始化。
FSM_StateName::STAND_UP 和 "STAND_UP" 分别表示状态名称和状态描述。
ini_foot_pos 是一个包含 4 个元素的向量，用于存储机器人的初始脚部位置。
通过将 checkSafeOrientation, checkPDesFoot 和 checkForceFeedForward 设置为 false，禁用在该状态下的安全检查。
*/
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
_ini_foot_pos(4){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

/*
onEnter：当进入该状态时，执行的初始化操作。
this->nextStateName = this->stateName;：默认将下一个状态设置为当前状态，表示没有状态转换。
this->transitionData.zero();：重置过渡数据。
iter = 0;：重置迭代计数器。
脚位置初始化：通过循环，获取每条腿的初始位置并存储在 _ini_foot_pos 中。
*/
template <typename T>
void FSM_State_StandUp<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;

  for(size_t leg(0); leg<4; ++leg){
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }
}


/*
run：在每个控制循环中执行的函数，用于逐步控制机器人站立。
如果机器人的类型是 MINI_CHEETAH，那么它将执行站立控制逻辑。
hMax = 0.25：定义机器人脚的最大高度变化（可能是站立时脚的升高）。
progress：定义进度，通过迭代计数器 iter 和控制时间步长 controller_dt 计算得出，范围在 0 到 1 之间。
循环：对每条腿施加控制，设定位置（pDes）、位置增益（kpCartesian）和速度增益（kdCartesian）。脚的目标高度由 progress 线性插值，从初始位置过渡到站立目标高度。
*/
/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_StandUp<T>::run() {

  if(this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
    T hMax = 0.25;
    T progress = 2 * iter * this->_data->controlParameters->controller_dt;

    if (progress > 1.){ progress = 1.; }

    for(int i = 0; i < 4; i++) {
      this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();
      this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();

      this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
      this->_data->_legController->commands[i].pDes[2] = 
        progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
    }
  }
}


/*
checkTransition：检查是否可以转换到其他状态。根据 control_mode 的值，决定下一个状态可以是哪一个：
K_STAND_UP：保持当前状态（不转换）。
K_BALANCE_STAND：转换到平衡站立状态。
K_LOCOMOTION：转换到运动状态。
K_VISION：进入视觉模式。
K_PASSIVE：进入被动模式。
默认行为：如果 control_mode 不在上述范围内，输出错误消息。
*/
/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_STAND_UP:
      break;
    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      break;


    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_PASSIVE << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}


/*
transition：执行状态转换的实际处理函数。
当下一个状态为 PASSIVE, BALANCE_STAND, LOCOMOTION 或 VISION 时，标记转换已完成。
如果转换状态异常，输出错误信息。
*/
/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_StandUp<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}


/**
 * onExit：当退出站立状态时调用。在这个实现中，退出时没有清理操作。
 */
/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_StandUp<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;
