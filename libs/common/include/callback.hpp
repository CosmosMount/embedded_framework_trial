#pragma once

#include <utility>

namespace core
{

// Non-owning callback. The bound object must outlive every registration that stores it.
template <typename Signature>
class callback;

template <typename Result, typename... Args>
class callback<Result(Args...)>
{
public:
    using entry_type = Result (*)(void*, Args...);

    constexpr callback() noexcept = default;

    template <Result (*Function)(Args...)>
    static constexpr callback bind() noexcept
    {
        return callback{nullptr, &invoke_function<Function>};
    }

    template <typename Object, Result (Object::*Method)(Args...)>
    static constexpr callback bind(Object* object) noexcept
    {
        return callback{object, &invoke_member<Object, Method>};
    }

    template <typename Object, Result (Object::*Method)(Args...) const>
    static constexpr callback bind(const Object* object) noexcept
    {
        return callback{const_cast<Object*>(object), &invoke_const_member<Object, Method>};
    }

    constexpr bool valid() const noexcept { return entry_ != nullptr; }
    constexpr explicit operator bool() const noexcept { return valid(); }

    Result operator()(Args... args) const
    {
        return entry_(context_, std::forward<Args>(args)...);
    }

    friend constexpr bool operator==(callback lhs, callback rhs) noexcept
    {
        return lhs.context_ == rhs.context_ && lhs.entry_ == rhs.entry_;
    }

private:
    constexpr callback(void* context, entry_type entry) noexcept : context_(context), entry_(entry) {}

    template <Result (*Function)(Args...)>
    static Result invoke_function(void*, Args... args)
    {
        return Function(std::forward<Args>(args)...);
    }

    template <typename Object, Result (Object::*Method)(Args...)>
    static Result invoke_member(void* context, Args... args)
    {
        return (static_cast<Object*>(context)->*Method)(std::forward<Args>(args)...);
    }

    template <typename Object, Result (Object::*Method)(Args...) const>
    static Result invoke_const_member(void* context, Args... args)
    {
        return (static_cast<const Object*>(context)->*Method)(std::forward<Args>(args)...);
    }

    void* context_ = nullptr;
    entry_type entry_ = nullptr;
};

} // namespace core
