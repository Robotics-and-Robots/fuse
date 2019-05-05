/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <fuse_core/constraint.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_core/variable.h>
#include <fuse_optimizers/variable_stamp_index.h>

#include <gtest/gtest.h>


/**
 * @brief Create a simple Variable implementation for testing
 */
class GenericVariable : public fuse_core::Variable
{
public:
  SMART_PTR_DEFINITIONS(GenericVariable);

  GenericVariable() :
    Variable(),
    data_{},
    uuid_{fuse_core::uuid::generate()}
  {}

  fuse_core::UUID uuid() const override { return uuid_; }

  size_t size() const override { return 1; }

  const double* data() const override { return &data_; }
  double* data() override { return &data_; }

  void print(std::ostream& stream = std::cout) const override {}

  fuse_core::Variable::UniquePtr clone() const override { return GenericVariable::make_unique(*this); }

protected:
  double data_;
  fuse_core::UUID uuid_;
};

/**
 * @brief Create a simple Constraint implementation for testing
 */
class GenericConstraint : public fuse_core::Constraint
{
public:
  SMART_PTR_DEFINITIONS(GenericConstraint);

  GenericConstraint(std::initializer_list<fuse_core::UUID> variable_uuids) : Constraint(variable_uuids) {}

  explicit GenericConstraint(const fuse_core::UUID& variable1) :
    fuse_core::Constraint{variable1} {}

  GenericConstraint(const fuse_core::UUID& variable1, const fuse_core::UUID& variable2) :
    fuse_core::Constraint{variable1, variable2} {}

  void print(std::ostream& stream = std::cout) const override {}

  ceres::CostFunction* costFunction() const override { return nullptr; }

  fuse_core::Constraint::UniquePtr clone() const override { return GenericConstraint::make_unique(*this); }
};


TEST(VariableStampIndex, Empty)
{
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
