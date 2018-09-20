#ifndef CSLIBS_MATH_ELLIPSE_HPP
#define CSLIBS_MATH_ELLIPSE_HPP

#include <cslibs_math/linear/vector.hpp>
#include <cslibs_math/linear/matrix.hpp>
#include <cslibs_math/common/equal.hpp>
#include <eigen3/Eigen/Jacobi>
#include <vector>
#include <limits>
#include <set>

namespace cslibs_math_2d {
template <typename T>
struct Ellipse
{
  Ellipse() {}
  Ellipse(T a, T b, T cx, T cy, T k):
    alpha(k)
  {
    axis(0) = a;
    axis(1) = b;
    center(0) = cx;
    center(1) = cy;
  }
  cslibs_math::linear::Vector<T,2> getPoint(T angle)
  {
    cslibs_math::linear::Vector<T,2> res;
    T x = axis(0) * std::cos(angle);
    T y = axis(1) * std::sin(angle);
    res(0) = std::cos(alpha) * x - std::sin(alpha) *y;
    res(1) = std::sin(alpha) * x + std::cos(alpha) *y;
    res += center;
    return res;
  }

  bool equals(const Ellipse<T>& other, double rel_diff = 0.01)
  {
    bool res = std::fabs(axis(0)/other.axis(0) - 1)    < 0.01
           && std::fabs(axis(1)/other.axis(1) - 1)     < 0.01
           && std::fabs(center(0)/other.center(0) - 1) < 0.01
           && std::fabs(center(1)/other.center(1) - 1) < 0.01
           && std::fabs(alpha/other.alpha - 1)         < 0.01;
    return res;
  }

  T alpha;
  cslibs_math::linear::Vector<T,2> axis;
  cslibs_math::linear::Vector<T,2> center;
};


template <typename T>
class EllipseFit
{
public:
  typedef Eigen::Matrix<T,Eigen::Dynamic,6> RegressionMatrix;
public:
  EllipseFit(){}

  void fit(const std::vector<cslibs_math::linear::Vector<T,2>>& points)
  {
    if(points.size() < 6){
      throw std::runtime_error("At least 6 points are required for an ellipse fit. Given are " + std::to_string(points.size()) + " points.");
    }
    RegressionMatrix reg_mat = getRegressionMatrix(points);
    Eigen::FullPivLU<Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic>> lu(reg_mat);
    hyper_params.data() = lu.kernel();
    Eigen::Matrix<T,Eigen::Dynamic, 1> test = reg_mat * hyper_params.data();
    cost = test.norm();
    ellipseFromHyper();

  }

  T cost;
  std::array<Ellipse<T>,4> solution;
  cslibs_math::linear::Matrix<T,6,1> hyper_params;

protected:
  RegressionMatrix getRegressionMatrix(const std::vector<cslibs_math::linear::Vector<T,2>>& points)
  {
    RegressionMatrix result = RegressionMatrix::Zero(points.size(),6);
    std::size_t row = 0;
    for(auto p : points){
      result(row,0) = p(0) * p (0);
      result(row,1) = p(1) * p (1);
      result(row,2) = p(0) * p (1);
      result(row,3) = p(0);
      result(row,4) = p(1);
      result(row,5) = T(1);
      ++row;
    }
    return result;
  }

  Eigen::Matrix<T,6,6> getMatrixM(const std::vector<cslibs_math::linear::Vector<T,2>>& points)
  {
    Eigen::Matrix<T,6,6> res = Eigen::Matrix<T,6,6>::Zero(6,6);
    for(auto p : points){
      Eigen::Matrix<T,6,1> xi;
      xi(0,0) = p(0) * p (0);
      xi(1,0) = p(1) * p (1);
      xi(2,0) = p(0) * p (1);
      xi(3,0) = p(0);
      xi(4,0) = p(1);
      xi(5,0) = T(1);
      res += xi * xi.transpose();
    }
    res /= points.size();
    return res;
  }


  void ellipseFromHyper()
  {
    const T& A = hyper_params(0,0);
    const T& C = hyper_params(1,0);
    const T& B = hyper_params(2,0);
    const T& D = hyper_params(3,0);
    const T& E = hyper_params(4,0);
    const T& F = hyper_params(5,0);
    if( A == 0  && B == 0 && C == 0 && D == 0 && E == 0 && F==0){
      throw std::runtime_error("Did not find non trivial solution. Ellipse fit not possible.");
    }
    Eigen::Matrix<T,3,3> M0;
    M0(0,0) = F;
    M0(0,1) = 0.5*D;
    M0(0,2) = 0.5*E;
    M0(1,0) = 0.5*D;
    M0(1,1) = A;
    M0(1,2) = 0.5*B;
    M0(2,0) = 0.5*E;
    M0(2,1) = 0.5*B;
    M0(2,2) = C;

    Eigen::Matrix<T,2,2> M;
    M(0,0) = A;
    M(0,1) = 0.5*B;
    M(1,0) = 0.5*B;
    M(1,1) = C;

    T detM0 = M0.determinant();
    T detM = M.determinant();
    T l1,l2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T,2,2> > eigensolver(M);
    if(eigensolver.info() == Eigen::Success){
      Eigen::Matrix<T,2,1> ev = eigensolver.eigenvalues();
      if(std::abs(ev(0,0) -A) <= std::abs(ev(0,0) -C) ){
        l1 = ev(0,0);
        l2 = ev(1,0);
//        std::cout << "l2 case" << std::endl;
      } else{
        l1 = ev(1,0);
        l2 = ev(0,0);
//        std::cout << "l1 case" << std::endl;
      }

      T a = std::sqrt(-detM0 /(detM * l1));
      T b = std::sqrt(-detM0 /(detM *l2));
      T cx = (B*E -2*C*D)/(4*A*C -B*B);
      T cy = (B*D -2*A*E)/(4*A*C -B*B);
      T t = (A-C)/B;
      T alpha = std::atan(1/t);
      if(t < 0){
        alpha += M_PI;
//        std::cout << "cotan case" << std::endl;
      }
      alpha *= 0.5;
      T alpha2 = alpha + M_PI/2;
      while(alpha2 > M_PI){
        alpha2 -= 2*M_PI;
      }
      while(alpha2 < -M_PI){
        alpha2 += 2*M_PI;
      }
      solution[0] = Ellipse<T>(a,b,cx,cy,alpha);
      solution[1] = Ellipse<T>(a,b,cx,cy,alpha2);
      solution[2] = Ellipse<T>(b,a,cx,cy,alpha);
      solution[3] = Ellipse<T>(b,a,cx,cy,alpha2);

    } else{
      std::cerr << "Cannot solve parametrization!" << std::endl;
      throw std::runtime_error("Cannot solve eigen problem.");
    }





  }
};
}
#endif // CSLIBS_MATH_2D_ELLIPSE_HPP
