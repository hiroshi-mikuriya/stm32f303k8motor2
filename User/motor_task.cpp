/// @file      User/motor_task.cpp
/// @author    Hiroshi Mikuriya
/// @copyright Copyright© 2021 Hiroshi Mikuriya
///
/// DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.

#include "gpio.hpp"
#include "main.h"

namespace
{
/// @brief モータータスククラス
class MotorTask
{
  InputPin swt_;                                      ///< モーター回転方向スイッチ
  OutputPin enable_;                                  ///< モータドライバ出力許可
  TIM_TypeDef *pwmTim_;                               ///< PWM制御ペリフェラル
  void (*setCompareValueP_)(TIM_TypeDef *, uint32_t); ///< PWM値設定関数（正方向）
  void (*setCompareValueN_)(TIM_TypeDef *, uint32_t); ///< PWM値設定関数（負方向）
  bool running_;

public:
  /// @brief デフォルトコンストラクタ
  MotorTask() : swt_({}), enable_({}), pwmTim_(0), setCompareValueP_(0), setCompareValueN_(0), running_(false) {}
  /// @brief プロパティ初期化
  /// @param [in] swt モーター回転方向スイッチ
  /// @param [in] enable モータドライバ出力許可
  /// @param [in] pwmTim PWM制御ペリフェラル
  /// @param [in] setCompareValueP PWM値設定関数（正方向）
  /// @param [in] setCompareValueN PWM値設定関数（負方向）
  void setProperty(                                      //
      InputPin const &swt,                               //
      OutputPin const &enable,                           //
      TIM_TypeDef *pwmTim,                               //
      void (*setCompareValueP)(TIM_TypeDef *, uint32_t), //
      void (*setCompareValueN)(TIM_TypeDef *, uint32_t)  //
  )
  {
    swt_ = swt;
    enable_ = enable;
    pwmTim_ = pwmTim;
    setCompareValueP_ = setCompareValueP;
    setCompareValueN_ = setCompareValueN;
  }

  /// @brief タスク処理実行 @param [in] res タスク共通リソース
  void run(TaskResource const &res)
  {
    running_ ? enable_.high() : enable_.low();
    if (swt_.level())
    {
      setCompareValueP_(pwmTim_, res.analog);
      setCompareValueN_(pwmTim_, 0);
    }
    else
    {
      setCompareValueP_(pwmTim_, 0);
      setCompareValueN_(pwmTim_, res.analog);
    }
  }
};
/// モータータスククラスインスタンス
MotorTask s_motorTask[COUNT_OF_MOTORS];
} // namespace

extern "C"
{
  /// @brief モータタスク初期化
  void motorTaskInit(void)
  {
    s_motorTask[0].setProperty(                      //
        InputPin{SWT1_GPIO_Port, SWT1_Pin},          //
        OutputPin{ENABLE_A_GPIO_Port, ENABLE_A_Pin}, //
        MOTOR_PWM_TIM,                               //
        LL_TIM_OC_SetCompareCH1,                     //
        LL_TIM_OC_SetCompareCH2                      //
    );
    s_motorTask[1].setProperty(                      //
        InputPin{SWT2_GPIO_Port, SWT2_Pin},          //
        OutputPin{ENABLE_B_GPIO_Port, ENABLE_B_Pin}, //
        MOTOR_PWM_TIM,                               //
        LL_TIM_OC_SetCompareCH3,                     //
        LL_TIM_OC_SetCompareCH4                      //
    );
    LL_TIM_CC_EnableChannel(MOTOR_PWM_TIM, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(MOTOR_PWM_TIM, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(MOTOR_PWM_TIM, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(MOTOR_PWM_TIM, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(MOTOR_PWM_TIM);
    LL_TIM_EnableAllOutputs(MOTOR_PWM_TIM);
  }
  /// @brief モータタスク
  /// @param [in] res タスク共有リソース
  void motorTaskProc(TaskResource *res)
  {
    for (int i = 0; i < COUNT_OF_MOTORS; ++i)
    {
      s_motorTask[i].run(res[i]);
    }
  }
}
