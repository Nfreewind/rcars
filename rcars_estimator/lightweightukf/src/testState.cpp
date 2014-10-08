#include "State.hpp"
#include "gtest/gtest.h"
#include <assert.h>

// The fixture for testing class ScalarElement.
class ScalarElementTest : public ::testing::Test {
 protected:
  ScalarElementTest() {
  }
  virtual ~ScalarElementTest() {
  }
  LightWeightUKF::ScalarElement testScalar1_;
  LightWeightUKF::ScalarElement testScalar2_;
  LightWeightUKF::ScalarElement::DiffVec difVec_;
  double testDouble1_ = 2.4;
  double testDouble2_ = 1.3;
};

// Test constructors
TEST_F(ScalarElementTest, constructors) {
  LightWeightUKF::ScalarElement testScalar1;
  ASSERT_EQ(testScalar1.x_,0.0);

  LightWeightUKF::ScalarElement testScalar2(testDouble1_);
  ASSERT_EQ(testScalar2.x_,testDouble1_);
}

// Test setIdentity
TEST_F(ScalarElementTest, setIdentity) {
  testScalar1_.setIdentity();
  ASSERT_EQ(testScalar1_.x_,0.0);
}

// Test plus and minus
TEST_F(ScalarElementTest, plusAndMinus) {
  testScalar1_.x_ = testDouble1_;
  difVec_(0) = testDouble2_;
  testScalar2_ = testScalar1_+difVec_;
  ASSERT_EQ(testScalar2_.x_,testDouble1_+testDouble2_);
  difVec_ = testScalar2_-testScalar1_;
  ASSERT_NEAR(difVec_(0),testDouble2_,1e-6);
}

// The fixture for testing class VectorElement.
class VectorElementTest : public ::testing::Test {
 protected:
  VectorElementTest() {
    testVector1_ << 2.1, -0.2, -1.9;
    testVector2_ << -10.6, 0.2, 25;
  }
  virtual ~VectorElementTest() {
  }
  LightWeightUKF::VectorElement testVectorElement1_;
  LightWeightUKF::VectorElement testVectorElement2_;
  LightWeightUKF::VectorElement::DiffVec difVec_;
  Eigen::Vector3d testVector1_;
  Eigen::Vector3d testVector2_;
};

// Test constructors
TEST_F(VectorElementTest, constructors) {
  LightWeightUKF::VectorElement testVectorElement1;
  ASSERT_EQ(testVectorElement1.x_(0),0.0);
  ASSERT_EQ(testVectorElement1.x_(1),0.0);
  ASSERT_EQ(testVectorElement1.x_(2),0.0);

  LightWeightUKF::VectorElement testVectorElement2(testVector1_);
  ASSERT_EQ(testVectorElement2.x_(0),testVector1_(0));
  ASSERT_EQ(testVectorElement2.x_(1),testVector1_(1));
  ASSERT_EQ(testVectorElement2.x_(2),testVector1_(2));
}

// Test setIdentity
TEST_F(VectorElementTest, setIdentity) {
  testVectorElement1_.setIdentity();
  ASSERT_EQ(testVectorElement1_.x_(0),0.0);
  ASSERT_EQ(testVectorElement1_.x_(1),0.0);
  ASSERT_EQ(testVectorElement1_.x_(2),0.0);
}

// Test plus and minus
TEST_F(VectorElementTest, plusAndMinus) {
  testVectorElement1_.x_ = testVector1_;
  difVec_ = testVector2_;
  testVectorElement2_ = testVectorElement1_+difVec_;
  ASSERT_EQ(testVectorElement2_.x_(0),testVector1_(0)+testVector2_(0));
  ASSERT_EQ(testVectorElement2_.x_(1),testVector1_(1)+testVector2_(1));
  ASSERT_EQ(testVectorElement2_.x_(2),testVector1_(2)+testVector2_(2));
  difVec_ = testVectorElement2_-testVectorElement1_;
  ASSERT_NEAR(difVec_(0),testVector2_(0),1e-6);
  ASSERT_NEAR(difVec_(1),testVector2_(1),1e-6);
  ASSERT_NEAR(difVec_(2),testVector2_(2),1e-6);
}

// The fixture for testing class QuaternionElement.
class QuaternionElementTest : public ::testing::Test {
 protected:
  QuaternionElementTest() {
    testQuatIdentity_ = rot::RotationQuaternionPD(1.0,0.0,0.0,0.0);
    testQuat1_ = rot::RotationQuaternionPD(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
    testQuat2_ = rot::RotationQuaternionPD(0.0,0.36,0.48,0.8);
  }
  virtual ~QuaternionElementTest() {
  }
  LightWeightUKF::QuaternionElement testQuaternionElement1_;
  LightWeightUKF::QuaternionElement testQuaternionElement2_;
  LightWeightUKF::QuaternionElement::DiffVec difVec_;
  rot::RotationQuaternionPD testQuatIdentity_;
  rot::RotationQuaternionPD testQuat1_;
  rot::RotationQuaternionPD testQuat2_;
  Eigen::Vector3d testVector2_;
};

// Test constructors
TEST_F(QuaternionElementTest, constructors) {
  LightWeightUKF::QuaternionElement testQuaternionElement1;
  ASSERT_EQ(testQuaternionElement1.x_.w(),1.0);
  ASSERT_EQ(testQuaternionElement1.x_.x(),0.0);
  ASSERT_EQ(testQuaternionElement1.x_.y(),0.0);
  ASSERT_EQ(testQuaternionElement1.x_.z(),0.0);

  LightWeightUKF::QuaternionElement testVectorElement2(testQuat1_);
  ASSERT_EQ(testVectorElement2.x_.w(),testQuat1_.w());
  ASSERT_EQ(testVectorElement2.x_.x(),testQuat1_.x());
  ASSERT_EQ(testVectorElement2.x_.y(),testQuat1_.y());
  ASSERT_EQ(testVectorElement2.x_.z(),testQuat1_.z());
}

// Test setIdentity
TEST_F(QuaternionElementTest, setIdentity) {
  testQuaternionElement1_.setIdentity();
  ASSERT_EQ(testQuaternionElement1_.x_.w(),1.0);
  ASSERT_EQ(testQuaternionElement1_.x_.x(),0.0);
  ASSERT_EQ(testQuaternionElement1_.x_.y(),0.0);
  ASSERT_EQ(testQuaternionElement1_.x_.z(),0.0);
}

// Test plus and minus
TEST_F(QuaternionElementTest, plusAndMinus) {
  // Plus - Minus
  testQuaternionElement1_.x_ = testQuat1_;
  difVec_ = testVector2_;
  testQuaternionElement2_ = testQuaternionElement1_+difVec_;
  difVec_ = testQuaternionElement2_-testQuaternionElement1_;
  ASSERT_NEAR(difVec_(0),testVector2_(0),1e-6);
  ASSERT_NEAR(difVec_(1),testVector2_(1),1e-6);
  ASSERT_NEAR(difVec_(2),testVector2_(2),1e-6);

  // Minus - Plus
  testQuaternionElement1_.x_ = testQuat1_;
  testQuaternionElement2_.x_ = testQuat2_;
  difVec_ = testQuaternionElement2_-testQuaternionElement1_;
  testQuaternionElement2_ = testQuaternionElement1_+difVec_;
  ASSERT_NEAR(testQuaternionElement2_.x_.w(),testQuat2_.w(),1e-6);
  ASSERT_NEAR(testQuaternionElement2_.x_.x(),testQuat2_.x(),1e-6);
  ASSERT_NEAR(testQuaternionElement2_.x_.y(),testQuat2_.y(),1e-6);
  ASSERT_NEAR(testQuaternionElement2_.x_.z(),testQuat2_.z(),1e-6);

  // Plus zero
  testQuaternionElement1_.x_ = testQuat1_;
  difVec_.setZero();
  testQuaternionElement2_ = testQuaternionElement1_+difVec_;
  ASSERT_NEAR(testQuaternionElement2_.x_.w(),testQuat1_.w(),1e-6);
  ASSERT_NEAR(testQuaternionElement2_.x_.x(),testQuat1_.x(),1e-6);
  ASSERT_NEAR(testQuaternionElement2_.x_.y(),testQuat1_.y(),1e-6);
  ASSERT_NEAR(testQuaternionElement2_.x_.z(),testQuat1_.z(),1e-6);

  // Minus same
  testQuaternionElement1_.x_ = testQuat1_;
  testQuaternionElement2_.x_ = testQuat1_;
  difVec_ = testQuaternionElement2_-testQuaternionElement1_;
  ASSERT_NEAR(difVec_(0),0.0,1e-6);
  ASSERT_NEAR(difVec_(1),0.0,1e-6);
  ASSERT_NEAR(difVec_(2),0.0,1e-6);
}

// The fixture for testing class VectorState
class VectorStateTest : public ::testing::Test {
 protected:
  VectorStateTest() {
    testVector1_ << 2.1, -0.2, -1.9, 0.2;
    testVector2_ << -10.6, 0.2, 25, -105.2;
  }
  virtual ~VectorStateTest() {
  }
  static const unsigned int D_ = 4;
  LightWeightUKF::VectorState<D_> testVectorState1_;
  LightWeightUKF::VectorState<D_> testVectorState2_;
  LightWeightUKF::VectorState<D_>::DiffVec difVec_;
  Eigen::Matrix<double,D_,1> testVector1_;
  Eigen::Matrix<double,D_,1> testVector2_;
};

// Test constructors
TEST_F(VectorStateTest, constructors) {
  LightWeightUKF::VectorState<D_> testVectorState1;
  for(int i=0;i<D_;i++){
    ASSERT_EQ(testVectorState1[i],0.0);
  }
}

// Test setIdentity
TEST_F(VectorStateTest, setIdentity) {
  testVectorState1_.setIdentity();
  for(int i=0;i<D_;i++){
    ASSERT_EQ(testVectorState1_[i],0.0);
  }
}

// Test plus and minus
TEST_F(VectorStateTest, plusAndMinus) {
  testVectorState1_.vector_ = testVector1_;
  difVec_ = testVector2_;
  testVectorState1_.boxplus(difVec_,testVectorState2_);
  for(int i=0;i<D_;i++){
    ASSERT_EQ(testVectorState2_[i],testVector1_[i]+testVector2_[i]);
  }
  testVectorState2_.boxminus(testVectorState1_,difVec_);
  for(int i=0;i<D_;i++){
    ASSERT_NEAR(difVec_[i],testVector2_[i],1e-6);
  }
}

// Test copy to array
TEST_F(VectorStateTest, copyToArray) {
  testVectorState1_.vector_ = testVector1_;
  double testArray[D_];
  testVectorState1_.copyToArray(testArray);
  for(int i=0;i<D_;i++){
    ASSERT_EQ(testVectorState1_[i],testArray[i]);
  }
}

// The fixture for testing class State
class StateTest : public ::testing::Test {
 protected:
  StateTest() {
    assert(V_>=Q_-1);
    testScalar1_[0] = 4.5;
    testScalar2_[0] = -17.34;
    for(int i=1;i<S_;i++){
      testScalar1_[i] = testScalar1_[i-1] + i*i*46.2;
      testScalar2_[i] = testScalar2_[i-1] - i*i*0.01;
    }
    testVector1_[0] << 2.1, -0.2, -1.9;
    testVector2_[0] << -10.6, 0.2, -105.2;
    for(int i=1;i<V_;i++){
      testVector1_[i] = testVector1_[i-1] + Eigen::Vector3d(0.3,10.9,2.3);
      testVector2_[i] = testVector2_[i-1] + Eigen::Vector3d(-1.5,12,1785.23);
    }
    testQuat1_[0] = rot::RotationQuaternionPD(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
    testQuat2_[0] = rot::RotationQuaternionPD(0.0,0.36,0.48,0.8);
    for(int i=1;i<Q_;i++){
      testQuat1_[i] = testQuat1_[i-1].boxPlus(testVector1_[i-1]);
      testQuat2_[i] = testQuat2_[i-1].boxPlus(testVector2_[i-1]);
    }
  }
  virtual ~StateTest() {
  }
  static const unsigned int S_ = 4;
  static const unsigned int V_ = 3;
  static const unsigned int Q_ = 2;
  LightWeightUKF::State<S_,V_,Q_> testState1_;
  LightWeightUKF::State<S_,V_,Q_> testState2_;
  LightWeightUKF::State<S_,V_,Q_>::DiffVec difVec_;
  double testScalar1_[S_];
  double testScalar2_[S_];
  Eigen::Vector3d testVector1_[V_];
  Eigen::Vector3d testVector2_[V_];
  rot::RotationQuaternionPD testQuat1_[Q_];
  rot::RotationQuaternionPD testQuat2_[Q_];
};

// Test constructors
TEST_F(StateTest, constructors) {
  LightWeightUKF::State<S_,V_,Q_> testState1;
  for(int i=0;i<S_;i++){
    ASSERT_EQ(testState1.scalarList[i].x_,0.0);
  }
  for(int i=0;i<V_;i++){
    ASSERT_EQ(testState1.vectorList[i].x_(0),0.0);
    ASSERT_EQ(testState1.vectorList[i].x_(1),0.0);
    ASSERT_EQ(testState1.vectorList[i].x_(2),0.0);
  }
  for(int i=0;i<Q_;i++){
    ASSERT_EQ(testState1.quaternionList[i].x_.w(),1.0);
    ASSERT_EQ(testState1.quaternionList[i].x_.x(),0.0);
    ASSERT_EQ(testState1.quaternionList[i].x_.y(),0.0);
    ASSERT_EQ(testState1.quaternionList[i].x_.z(),0.0);
  }
}

// Test setIdentity
TEST_F(StateTest, setIdentity) {
  testState1_.setIdentity();
  for(int i=0;i<S_;i++){
    ASSERT_EQ(testState1_.scalarList[i].x_,0.0);
  }
  for(int i=0;i<V_;i++){
    ASSERT_EQ(testState1_.vectorList[i].x_(0),0.0);
    ASSERT_EQ(testState1_.vectorList[i].x_(1),0.0);
    ASSERT_EQ(testState1_.vectorList[i].x_(2),0.0);
  }
  for(int i=0;i<Q_;i++){
    ASSERT_EQ(testState1_.quaternionList[i].x_.w(),1.0);
    ASSERT_EQ(testState1_.quaternionList[i].x_.x(),0.0);
    ASSERT_EQ(testState1_.quaternionList[i].x_.y(),0.0);
    ASSERT_EQ(testState1_.quaternionList[i].x_.z(),0.0);
  }
}

// Test isNearIdentity
TEST_F(StateTest, isNearIdentity) {
  // Test with identity
  testState1_.setIdentity();
  ASSERT_TRUE(testState1_.isNearIdentity(1e-8));

  // Test with non-identity
  for(int i=0;i<S_;i++){
    testState1_.scalarList[i].x_ = testScalar1_[i];
  }
  for(int i=0;i<V_;i++){
    testState1_.vectorList[i].x_ = testVector1_[i];
  }
  for(int i=0;i<Q_;i++){
    testState1_.quaternionList[i].x_ = testQuat1_[i];
  }
  ASSERT_FALSE(testState1_.isNearIdentity(1e-8));
}

// Test plus and minus
TEST_F(StateTest, plusAndMinus) {
  for(int i=0;i<S_;i++){
    testState1_.scalarList[i].x_ = testScalar1_[i];
  }
  for(int i=0;i<V_;i++){
    testState1_.vectorList[i].x_ = testVector1_[i];
  }
  for(int i=0;i<Q_;i++){
    testState1_.quaternionList[i].x_ = testQuat1_[i];
  }
  for(int i=0;i<S_;i++){
    testState2_.scalarList[i].x_ = testScalar2_[i];
  }
  for(int i=0;i<V_;i++){
    testState2_.vectorList[i].x_ = testVector2_[i];
  }
  for(int i=0;i<Q_;i++){
    testState2_.quaternionList[i].x_ = testQuat2_[i];
  }
  testState2_.boxminus(testState1_,difVec_);
  unsigned int index=0;
  for(int i=0;i<S_;i++){
    ASSERT_EQ(difVec_(index),testScalar2_[i]-testScalar1_[i]);
    index ++;
  }
  for(int i=0;i<V_;i++){
    ASSERT_EQ(difVec_(index),testVector2_[i](0)-testVector1_[i](0));
    index ++;
    ASSERT_EQ(difVec_(index),testVector2_[i](1)-testVector1_[i](1));
    index ++;
    ASSERT_EQ(difVec_(index),testVector2_[i](2)-testVector1_[i](2));
    index ++;
  }
  for(int i=0;i<Q_;i++){
    ASSERT_EQ(difVec_(index),testQuat2_[i].boxMinus(testQuat1_[i])(0));
    index ++;
    ASSERT_EQ(difVec_(index),testQuat2_[i].boxMinus(testQuat1_[i])(1));
    index ++;
    ASSERT_EQ(difVec_(index),testQuat2_[i].boxMinus(testQuat1_[i])(2));
    index ++;
  }
  testState1_.boxplus(difVec_,testState2_);
  for(int i=0;i<S_;i++){
    ASSERT_NEAR(testState2_.scalarList[i].x_,testScalar2_[i],1e-6);
  }
  for(int i=0;i<V_;i++){
    ASSERT_NEAR(testState2_.vectorList[i].x_(0),testVector2_[i](0),1e-6);
    ASSERT_NEAR(testState2_.vectorList[i].x_(1),testVector2_[i](1),1e-6);
    ASSERT_NEAR(testState2_.vectorList[i].x_(2),testVector2_[i](2),1e-6);
  }
  for(int i=0;i<Q_;i++){
    ASSERT_TRUE(testState2_.quaternionList[i].x_.isNear(testQuat2_[i],1e-6));
  }
}

// Test copy to array
TEST_F(StateTest, copyToArray) {
  for(int i=0;i<S_;i++){
    testState1_.scalarList[i].x_ = testScalar1_[i];
  }
  for(int i=0;i<V_;i++){
    testState1_.vectorList[i].x_ = testVector1_[i];
  }
  for(int i=0;i<Q_;i++){
    testState1_.quaternionList[i].x_ = testQuat1_[i];
  }
  double testArray[S_+3*V_+4*Q_];
  testState1_.copyToArray(testArray);
  unsigned int index = 0;
  for(int i=0;i<S_;i++){
    ASSERT_EQ(testState1_.scalarList[i].x_,testArray[index]);
    index++;
  }
  for(int i=0;i<V_;i++){
    ASSERT_EQ(testState1_.vectorList[i].x_(0),testArray[index]);
    index++;
    ASSERT_EQ(testState1_.vectorList[i].x_(1),testArray[index]);
    index++;
    ASSERT_EQ(testState1_.vectorList[i].x_(2),testArray[index]);
    index++;
  }
  for(int i=0;i<Q_;i++){
    ASSERT_EQ(testState1_.quaternionList[i].x_.w(),testArray[index]);
    index++;
    ASSERT_EQ(testState1_.quaternionList[i].x_.x(),testArray[index]);
    index++;
    ASSERT_EQ(testState1_.quaternionList[i].x_.y(),testArray[index]);
    index++;
    ASSERT_EQ(testState1_.quaternionList[i].x_.z(),testArray[index]);
    index++;
  }
}

// Test accessors
TEST_F(StateTest, accessors) {
  for(int i=0;i<S_;i++){
    testState1_.scalarList[i].x_ = testScalar1_[i];
  }
  for(int i=0;i<V_;i++){
    testState1_.vectorList[i].x_ = testVector1_[i];
  }
  for(int i=0;i<Q_;i++){
    testState1_.quaternionList[i].x_ = testQuat1_[i];
  }
  for(int i=0;i<S_;i++){
    testState1_.s(i) = testScalar1_[i];
  }
  for(int i=0;i<V_;i++){
    testState1_.v(i)(0) = testVector1_[i](0);
    testState1_.v(i)(1) = testVector1_[i](1);
    testState1_.v(i)(2) = testVector1_[i](2);
  }
  for(int i=0;i<Q_;i++){
    ASSERT_TRUE(testState1_.q(i).isNear(testQuat1_[i],1e-6));
  }
}

int main(int argc, char **argv) {
//  LightWeightUKF::NewState<LightWeightUKF::ScalarElement,LightWeightUKF::QuaternionElement> aa;
//  std::cout << aa.D_ << std::endl;
////  aa.x<1>().x_ = rot::RotationQuaternionPD();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
