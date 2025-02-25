/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "denso_robot_core/denso_controller_rc8.h"

namespace denso_robot_core {
DensoControllerRC8::DensoControllerRC8(
  rclcpp::Node::SharedPtr& node, const std::string& name, const int* mode,
  const std::string& ip_address, const rclcpp::Duration dt)
  : DensoController(node, name, mode, ip_address, dt)
{
}

DensoControllerRC8::~DensoControllerRC8()
{
}

HRESULT DensoControllerRC8::AddController()
{
  static const std::string CTRL_CONNECT_OPTION[BCAP_CONTROLLER_CONNECT_ARGS]
    = { "", "CaoProv.DENSO.VRC", "localhost", "" };

  HRESULT hr = E_FAIL;
  int srvs, argc;

  if (m_duration.seconds() != 0.008) {
    RCLCPP_FATAL(
      rclcpp::get_logger(m_node->get_name()),
      "Invalid argument value [bcap_slave_control_cycle_msec]: %.1f",
      m_duration.seconds() * 1000);
    return E_INVALIDARG;
  }

  for (srvs = DensoBase::SRV_MIN; srvs <= DensoBase::SRV_MAX; srvs++)
  {
    std::stringstream ss;
    std::string strTmp;
    VARIANT_Ptr vntRet(new VARIANT());
    VARIANT_Vec vntArgs;

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_CONTROLLER_CONNECT_ARGS; argc++) {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      vntTmp->vt = VT_BSTR;

      if (argc == 0) {
        strTmp = "";
        if (m_name != "") {
          ss << m_node->get_namespace() << m_name << srvs;
          strTmp = ss.str();
        }
      } else {
        strTmp = CTRL_CONNECT_OPTION[argc];
      }

      vntTmp->bstrVal = ConvertStringToBSTR(strTmp);

      vntArgs.push_back(*vntTmp.get());
    }

    hr = m_vecService[srvs]->ExecFunction(ID_CONTROLLER_CONNECT, vntArgs, vntRet);
    if (FAILED(hr)) {
      break;
    }

    m_vecHandle.push_back(vntRet->ulVal);
  }

  hr = GetControllerVersion(m_ctrlVer);
  if (FAILED(hr)) {
    RCLCPP_FATAL(
      rclcpp::get_logger(m_node->get_name()), "Failed to get controller version. (%X)", hr);
  }

  return hr;
}

HRESULT DensoControllerRC8::AddRobot(XMLElement * xmlElem)
{
  std::size_t objs;
  HRESULT hr;

  Name_Vec vecName;
  hr = DensoBase::GetObjectNames(ID_CONTROLLER_GETROBOTNAMES, vecName);
  if (SUCCEEDED(hr)) {
    for (objs = 0; objs < vecName.size(); objs++) {
      Handle_Vec vecHandle;
      hr = DensoBase::AddObject(ID_CONTROLLER_GETROBOT, vecName[objs], vecHandle);
      if (FAILED(hr)) {
        break;
      }

      DensoRobot_Ptr rob(
        new DensoRobotRC8(m_node, this, m_vecService, vecHandle, vecName[objs], m_mode));
      hr = rob->InitializeBCAP(xmlElem);
      if (FAILED(hr)) {
        break;
      }

      m_vecRobot.push_back(rob);
    }
  }

  return hr;
}

HRESULT DensoControllerRC8::get_Robot(int index, DensoRobotRC8_Ptr * robot)
{
  if (robot == NULL) {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), m_vecRobot.begin(), m_vecRobot.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, index, &pBase);
  if (SUCCEEDED(hr)) {
    *robot = std::dynamic_pointer_cast<DensoRobotRC8>(pBase);
  }

  return hr;
}

/**
 * Reset the STO(Safe Torque Off) state.
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoControllerRC8::ExecResetStoState()
{
  /*
   * NOTE:
   *
   * RC8: Do nothing.
   * RC8A: Do nothing because "ClearError" can reset the STO together.
   */
  return 0;
}

}  // namespace denso_robot_core
