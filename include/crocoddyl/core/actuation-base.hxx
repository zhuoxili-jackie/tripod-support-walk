///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2025, LAAS-CNRS, University of Edinburgh,
//                          Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

namespace crocoddyl {

template <typename Scalar>
ActuationModelAbstractTpl<Scalar>::ActuationModelAbstractTpl(
    std::shared_ptr<StateAbstract> state, const std::size_t nu)
    : nu_(nu),
      state_(state),
      u_lb_(MathBase::VectorXs::Constant(
          nu, -std::numeric_limits<Scalar>::infinity())),
      u_ub_(MathBase::VectorXs::Constant(
          nu, std::numeric_limits<Scalar>::infinity())) {}

template <typename Scalar>
std::shared_ptr<ActuationDataAbstractTpl<Scalar> >
ActuationModelAbstractTpl<Scalar>::createData() {
  return std::allocate_shared<ActuationDataAbstract>(
      Eigen::aligned_allocator<ActuationDataAbstract>(), this);
}

template <typename Scalar>
void ActuationModelAbstractTpl<Scalar>::calc(
    const std::shared_ptr<ActuationDataAbstract>&,
    const Eigen::Ref<const VectorXs>&) {}

template <typename Scalar>
void ActuationModelAbstractTpl<Scalar>::calcDiff(
    const std::shared_ptr<ActuationDataAbstract>&,
    const Eigen::Ref<const VectorXs>&) {}

template <typename Scalar>
void ActuationModelAbstractTpl<Scalar>::torqueTransform(
    const std::shared_ptr<ActuationDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty(
        "Invalid argument: " << "x has wrong dimension (it should be " +
                                    std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty(
        "Invalid argument: " << "u has wrong dimension (it should be " +
                                    std::to_string(nu_) + ")");
  }
  calc(data, x, u);
  calcDiff(data, x, u);
  data->Mtau = pseudoInverse(data->dtau_du);
}

template <typename Scalar>
std::size_t ActuationModelAbstractTpl<Scalar>::get_nu() const {
  return nu_;
}

template <typename Scalar>
const std::shared_ptr<StateAbstractTpl<Scalar> >&
ActuationModelAbstractTpl<Scalar>::get_state() const {
  return state_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs&
ActuationModelAbstractTpl<Scalar>::get_u_lb() const {
  return u_lb_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs&
ActuationModelAbstractTpl<Scalar>::get_u_ub() const {
  return u_ub_;
}

template <typename Scalar>
void ActuationModelAbstractTpl<Scalar>::set_u_lb(const VectorXs& u_lb) {
  if (static_cast<std::size_t>(u_lb.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "control lower bound has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }
  u_lb_ = u_lb;
  for (std::size_t i = 0; i < nu_; ++i) {
    if (u_lb_[i] <= -std::numeric_limits<Scalar>::max()) {
      u_lb_[i] = -std::numeric_limits<Scalar>::infinity();
    }
  }
}

template <typename Scalar>
void ActuationModelAbstractTpl<Scalar>::set_u_ub(const VectorXs& u_ub) {
  if (static_cast<std::size_t>(u_ub.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "control upper bound has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }
  u_ub_ = u_ub;
  for (std::size_t i = 0; i < nu_; ++i) {
    if (u_ub_[i] >= std::numeric_limits<Scalar>::max()) {
      u_ub_[i] = std::numeric_limits<Scalar>::infinity();
    }
  }
}

template <typename Scalar>
std::ostream& operator<<(std::ostream& os,
                         const ActuationModelAbstractTpl<Scalar>& model) {
  model.print(os);
  return os;
}

template <typename Scalar>
void ActuationModelAbstractTpl<Scalar>::print(std::ostream& os) const {
  os << boost::core::demangle(typeid(*this).name());
}

}  // namespace crocoddyl
