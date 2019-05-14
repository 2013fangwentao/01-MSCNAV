/*
** filter.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/include
**
** Made by little fang
** Login   <fangwentao>
**
** base class of kalman filter
** Started on  Tue Dec 17 下午3:02:10 2018 little fang
** Last update Wed May 14 下午5:54:41 2019 little fang
*/

#ifndef FILTER_H_
#define FILTER_H_
#include "navstruct.h"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace mscnav
{
    
class KalmanFilter
{
public:
  KalmanFilter();
  ~KalmanFilter();

private:
  Eigen::MatrixXd state_cov_;
//  Eigen::MatrixXd 
};

} // namespace mscnav
#endif /* !FILTER_H_ */
