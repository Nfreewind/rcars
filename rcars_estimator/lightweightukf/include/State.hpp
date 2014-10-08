/*
 * State.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef STATE_HPP_
#define STATE_HPP_

#include <Eigen/Dense>
#include <iostream>
#include "kindr/rotations/RotationEigen.hpp"
namespace rot = kindr::rotations::eigen_impl;

namespace LightWeightUKF{

template<typename DERIVED,typename Base,unsigned int D>
class ElementBase{
 public:
  static const unsigned int D_ = D;
  typedef Eigen::Matrix<double,D_,1> DiffVec;
  Base x_;
  ElementBase(){};
  ElementBase(const Base& x){
    x_ = x;
  }
  virtual ~ElementBase(){};
  virtual void setIdentity() = 0;
  virtual DERIVED operator +(const DiffVec& vecIn) const = 0;
  virtual DiffVec operator -(const DERIVED& elementIn) const = 0;
};

class ScalarElement: public ElementBase<ScalarElement,double,1> {
 public:
  ScalarElement(const double& x):ElementBase(x){};
  ScalarElement(){setIdentity();};
  void setIdentity(){
    x_ = 0.0;
  }
  ScalarElement operator +(const DiffVec& vecIn) const{
    return ScalarElement(x_ + vecIn(0));
  }
  DiffVec operator -(const ScalarElement& elementIn) const{
    return (ScalarElement::DiffVec() << x_ - elementIn.x_).finished();
  }
};

class VectorElement: public ElementBase<VectorElement,Eigen::Vector3d,3> {
 public:
  VectorElement(const Eigen::Vector3d& x):ElementBase(x){};
  VectorElement(){setIdentity();};
  void setIdentity(){
    x_.setZero();
  }
  VectorElement operator +(const DiffVec& vecIn) const{
    return VectorElement(x_ + vecIn);
  }
  DiffVec operator -(const VectorElement& elementIn) const{
    return x_ - elementIn.x_;
  }
};

class QuaternionElement: public ElementBase<QuaternionElement,rot::RotationQuaternionPD,3> {
 public:
  QuaternionElement(const rot::RotationQuaternionPD& x):ElementBase(x){};
  QuaternionElement(){setIdentity();};
  void setIdentity(){
    x_.setIdentity();
  }
  QuaternionElement operator +(const DiffVec& vecIn) const{
    return QuaternionElement(x_.boxPlus(vecIn));
  }
  DiffVec operator -(const QuaternionElement& elementIn) const{
    return x_.boxMinus(elementIn.x_);
  }
};

template<unsigned int S, unsigned int V, unsigned int Q>
class State{
 public:
  static const unsigned int D_ = S*ScalarElement::D_+V*VectorElement::D_+Q*QuaternionElement::D_;
  static const unsigned int Ds_ = S*ScalarElement::D_;
  static const unsigned int Dv_ = V*VectorElement::D_;
  static const unsigned int Dq_ = Q*QuaternionElement::D_;
  typedef Eigen::Matrix<double,D_,1> DiffVec;
  typedef Eigen::Matrix<double,D_,D_> CovMat;
  State(){
    t_ = 0.0;
  };
  double t_;
  ScalarElement scalarList[S];
  VectorElement vectorList[V];
  QuaternionElement quaternionList[Q];
  void boxplus(const DiffVec& vecIn, State<S,V,Q>& stateOut) const{
    unsigned int index = 0;
    for(unsigned int i=0;i<S;i++){
      stateOut.scalarList[i] = scalarList[i]+vecIn.block(index,0,ScalarElement::D_,1);
      index += ScalarElement::D_;
    }
    for(unsigned int i=0;i<V;i++){
      stateOut.vectorList[i] = vectorList[i]+vecIn.block(index,0,VectorElement::D_,1);
      index += VectorElement::D_;
    }
    for(unsigned int i=0;i<Q;i++){
      stateOut.quaternionList[i] = quaternionList[i]+vecIn.block(index,0,QuaternionElement::D_,1);
      index += QuaternionElement::D_;
    }
    stateOut.t_ = t_;
  }
  void boxminus(const State<S,V,Q>& stateIn, DiffVec& vecOut) const{
    unsigned int index = 0;
    for(unsigned int i=0;i<S;i++){
      vecOut.block(index,0,ScalarElement::D_,1) = scalarList[i]-stateIn.scalarList[i];
      index += ScalarElement::D_;
    }
    for(unsigned int i=0;i<V;i++){
      vecOut.block(index,0,VectorElement::D_,1) = vectorList[i]-stateIn.vectorList[i];
      index += VectorElement::D_;
    }
    for(unsigned int i=0;i<Q;i++){
      vecOut.block(index,0,QuaternionElement::D_,1) = quaternionList[i]-stateIn.quaternionList[i];
      index += QuaternionElement::D_;
    }
  }
  void print() const{
    std::cout << "Scalars:" << std::endl;
    for(unsigned int i=0;i<S;i++){
      std::cout << s(i) << std::endl;
    }
    std::cout << "Vectors:" << std::endl;
    for(unsigned int i=0;i<V;i++){
      std::cout << v(i).transpose() << std::endl;
    }
    std::cout << "Quaternions:" << std::endl;
    for(unsigned int i=0;i<Q;i++){
      std::cout << q(i) << std::endl;
    }
  }
  void copyToArray(double* arr) const{
    unsigned int index = 0;
    for(unsigned int i=0;i<S;i++){
      arr[index] = s(i);
      index++;
    }
    for(unsigned int i=0;i<V;i++){
      arr[index] = v(i)(0);
      index++;
      arr[index] = v(i)(1);
      index++;
      arr[index] = v(i)(2);
      index++;
    }
    for(unsigned int i=0;i<Q;i++){
      arr[index] = q(i).w();
      index++;
      arr[index] = q(i).x();
      index++;
      arr[index] = q(i).y();
      index++;
      arr[index] = q(i).z();
      index++;
    }
  }
  void setIdentity(){
    for(unsigned int i=0;i<S;i++){
      scalarList[i].setIdentity();
    }
    for(unsigned int i=0;i<V;i++){
      vectorList[i].setIdentity();
    }
    for(unsigned int i=0;i<Q;i++){
      quaternionList[i].setIdentity();
    }
  }
  bool isNearIdentity(double th) const{
    State identity;
    identity.setIdentity();
    State::DiffVec vec;
    this->boxminus(identity,vec);
    return vec.norm()<th;
  }
  void fix(){
    for(unsigned int i=0;i<Q;i++){
      q(i).fix();
    }
  }
  const double& s(unsigned int i) const{
    assert(i<S);
    return scalarList[i].x_;
  };
  double& s(unsigned int i) {
    assert(i<S);
    return scalarList[i].x_;
  };
  const Eigen::Matrix<double,3,1>& v(unsigned int i) const{
    assert(i<V);
    return vectorList[i].x_;
  };
  Eigen::Matrix<double,3,1>& v(unsigned int i) {
    assert(i<V);
    return vectorList[i].x_;
  };
  const rot::RotationQuaternionPD& q(unsigned int i) const{
    assert(i<Q);
    return quaternionList[i].x_;
  };
  rot::RotationQuaternionPD& q(unsigned int i) {
    assert(i<Q);
    return quaternionList[i].x_;
  };
};

template<unsigned int N>
class VectorState{
 public:
  static const unsigned int D_ = N;
  typedef Eigen::Matrix<double,D_,1> DiffVec;
  typedef Eigen::Matrix<double,D_,D_> CovMat;
  VectorState(){
    vector_.setZero();
    t_ = 0.0;
  };
  Eigen::Matrix<double,D_,1> vector_;
  double t_;
  void boxplus(const DiffVec& vecIn, VectorState<N>& stateOut) const{
    stateOut.vector_ = vector_+vecIn;
    stateOut.t_ = t_;
  }
  void boxminus(const VectorState<N>& stateIn, DiffVec& vecOut) const{
    vecOut = vector_-stateIn.vector_;
  }
  void print() const{
    std::cout << "Vector:" << vector_.transpose() << std::endl;
  }
  void copyToArray(double* arr) const{
    for(unsigned int i=0;i<D_;i++){
      arr[i] = vector_(i);
    }
  }
  void setIdentity(){
    vector_.setZero();
  }
  const double& operator[](unsigned int i) const{
    assert(i<D_);
    return vector_(i);
  };
  double& operator[](unsigned int i){
    assert(i<D_);
    return vector_(i);
  };
};

//template<typename T, typename... Arguments>
//class NewState{
// public:
//  static const unsigned int D_ = T::D_ + NewState<Arguments...>::D_;
//  T x_;
//  NewState<Arguments...> y_;
//
//  template<int i>
//  struct function_{
//    typedef typename NewState<Arguments...>::template function_<i-1>::argType argType;
//  };
//
//  template<>
//  struct function_<0>{
//    typedef T argType;
//  };
//
////  template<int i>
////  using argType = typename NewState<Arguments...>::template argType<i-1>;
////  // TODO: reuires specialization for i=0
////
////  template<int i>
////  argType<i>& x(){
////    if(i==0) return x_;
////    else return y_.x<i-1>();
////  }
//};
//
//template<typename T>
//class NewState<T>{
// public:
//  static const unsigned int D_ = T::D_;
//  T x_;
//
//  template<int i>
//  struct function_{
//    typedef T argType;
//  };
//
////  template<int i>
////  using argType = T;
////
////  template<int i>
////  argType<i>& x(){
////    assert(i==0);
////    return x_;
////  }
//};

}

#endif /* STATE_HPP_ */
