#pragma once

#include "robot_config.hpp"

#include "djimotors.hpp"
#include "dmmotors.hpp"
#include "lkmotors.hpp"

namespace robot::devices::motors
{

template <robot::motors::model Model>
struct model_traits;

template <>
struct model_traits<robot::motors::model::dji_m2006>
{
    using type = ::motors::m2006;
};

template <>
struct model_traits<robot::motors::model::dji_m3508>
{
    using type = ::motors::m3508;
};

template <>
struct model_traits<robot::motors::model::dji_gm6020>
{
    using type = ::motors::gm6020;
};

template <>
struct model_traits<robot::motors::model::dji_xroll>
{
    using type = ::motors::xroll;
};

template <>
struct model_traits<robot::motors::model::dm_dm4310>
{
    using type = ::motors::dm4310;
};

template <>
struct model_traits<robot::motors::model::dm_dm8009p>
{
    using type = ::motors::dm8009p;
};

template <>
struct model_traits<robot::motors::model::lk_lk8016>
{
    using type = ::motors::lk8016;
};

template <>
struct model_traits<robot::motors::model::lk_lk9025>
{
    using type = ::motors::lk9025;
};

template <robot::motors::model Model>
using model_type_t = typename model_traits<Model>::type;

} // namespace robot::devices::motors
