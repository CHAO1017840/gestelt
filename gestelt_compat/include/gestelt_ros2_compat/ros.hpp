#ifndef GESTELT_ROS2_COMPAT_ROS_HPP_
#define GESTELT_ROS2_COMPAT_ROS_HPP_

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <typeindex>
#include <type_traits>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gestelt_ros2_compat/types.hpp"

namespace gestelt_ros2_compat
{
inline rclcpp::Node::SharedPtr & global_node()
{
  static rclcpp::Node::SharedPtr node;
  return node;
}

inline rclcpp::executors::MultiThreadedExecutor::SharedPtr & global_executor()
{
  static rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  return executor;
}

inline size_t & executor_thread_count()
{
  static size_t thread_count{0};
  return thread_count;
}

inline std::atomic<bool> & executor_spinning()
{
  static std::atomic<bool> spinning{false};
  return spinning;
}

inline rclcpp::CallbackGroup::SharedPtr & default_callback_group()
{
  static rclcpp::CallbackGroup::SharedPtr group;
  return group;
}

inline rclcpp::Logger logger()
{
  return global_node() ? global_node()->get_logger() : rclcpp::get_logger("gestelt");
}

template<typename SharedPtrT>
void preserve_after_shutdown(const SharedPtrT & pointer)
{
  if (!rclcpp::ok() && pointer) {
    // Destruction after context shutdown can race with DDS teardown. At process
    // exit it is safe to retain the handle until the operating system reclaims it.
    static_cast<void>(new SharedPtrT(pointer));
  }
}

inline void configure_executor(size_t thread_count)
{
  if (!global_node()) {
    return;
  }
  if (global_executor() && executor_thread_count() == thread_count) {
    return;
  }
  if (executor_spinning()) {
    throw std::runtime_error("Cannot resize the executor while it is spinning");
  }
  global_executor().reset();
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), std::max<size_t>(1, thread_count), false);
  executor->add_node(global_node());
  global_executor() = std::move(executor);
  executor_thread_count() = thread_count;
}

[[noreturn]] inline void exit_after_shutdown()
{
  std::fflush(nullptr);
  std::_Exit(EXIT_SUCCESS);
}

inline std::string normalize_parameter_name(const std::string & name)
{
  std::string normalized = name;
  for (char & c : normalized) {
    if (c == '/') {
      c = '.';
    }
  }
  while (!normalized.empty() && normalized.front() == '.') {
    normalized.erase(normalized.begin());
  }
  return normalized;
}
}  // namespace gestelt_ros2_compat

namespace ros
{
class Duration
{
public:
  Duration() = default;
  explicit Duration(double seconds)
  : duration_(rclcpp::Duration::from_seconds(seconds)) {}
  explicit Duration(int seconds)
  : duration_(rclcpp::Duration::from_seconds(static_cast<double>(seconds))) {}
  explicit Duration(rclcpp::Duration duration)
  : duration_(duration) {}

  static Duration fromNSec(int64_t nanoseconds)
  {
    Duration result;
    result.duration_ = rclcpp::Duration::from_nanoseconds(nanoseconds);
    return result;
  }

  double toSec() const {return duration_.seconds();}
  int64_t toNSec() const {return duration_.nanoseconds();}
  bool isZero() const {return duration_.nanoseconds() == 0;}
  void sleep() const {rclcpp::sleep_for(duration_.to_chrono<std::chrono::nanoseconds>());}
  rclcpp::Duration get() const {return duration_;}
  Duration operator+(const Duration & other) const
  {
    return fromNSec(toNSec() + other.toNSec());
  }
  Duration operator-(const Duration & other) const
  {
    return fromNSec(toNSec() - other.toNSec());
  }

private:
  rclcpp::Duration duration_{0, 0};
};

class Time
{
public:
  Time()
  : time_(0, 0, RCL_ROS_TIME) {}
  explicit Time(int seconds)
  : time_(static_cast<int64_t>(seconds) * 1000000000LL, RCL_ROS_TIME) {}
  explicit Time(double seconds)
  : time_(static_cast<int64_t>(seconds * 1000000000.0), RCL_ROS_TIME) {}
  explicit Time(const builtin_interfaces::msg::Time & timestamp)
  : time_(timestamp) {}
  explicit Time(const rclcpp::Time & timestamp)
  : time_(timestamp) {}

  static Time now()
  {
    if (gestelt_ros2_compat::global_node()) {
      return Time(gestelt_ros2_compat::global_node()->now());
    }
    return Time(rclcpp::Clock(RCL_ROS_TIME).now());
  }

  static Time fromSec(double seconds) {return Time(seconds);}
  static Time fromNSec(int64_t nanoseconds)
  {
    Time result;
    result.time_ = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
    return result;
  }
  double toSec() const {return time_.seconds();}
  int64_t toNSec() const {return time_.nanoseconds();}
  bool isZero() const {return time_.nanoseconds() == 0;}
  rclcpp::Time get() const {return time_;}
  operator builtin_interfaces::msg::Time() const {return time_;}
  operator rclcpp::Time() const {return time_;}

  Duration operator-(const Time & other) const
  {
    return Duration::fromNSec(time_.nanoseconds() - other.time_.nanoseconds());
  }
  Time operator-(const Duration & other) const
  {
    return Time::fromNSec(time_.nanoseconds() - other.toNSec());
  }
  Time operator+(const Duration & other) const
  {
    return Time::fromNSec(time_.nanoseconds() + other.toNSec());
  }
  bool operator<(const Time & other) const {return time_ < other.time_;}
  bool operator<=(const Time & other) const {return time_ <= other.time_;}
  bool operator>(const Time & other) const {return time_ > other.time_;}
  bool operator>=(const Time & other) const {return time_ >= other.time_;}
  bool operator==(const Time & other) const {return time_ == other.time_;}
  bool operator!=(const Time & other) const {return time_ != other.time_;}

private:
  rclcpp::Time time_;
};

inline bool operator>(const Duration & lhs, const Duration & rhs)
{
  return lhs.toNSec() > rhs.toNSec();
}
inline bool operator<(const Duration & lhs, const Duration & rhs)
{
  return lhs.toNSec() < rhs.toNSec();
}

struct TimerEvent {};
struct TransportHints
{
  TransportHints & tcpNoDelay(bool = true) {return *this;}
  TransportHints & udp() {return *this;}
};

class Publisher
{
public:
  Publisher() = default;

  template<typename MessageT>
  static Publisher create(typename rclcpp::Publisher<MessageT>::SharedPtr publisher)
  {
    Publisher result;
    result.model_ = std::make_shared<Model<MessageT>>(std::move(publisher));
    return result;
  }

  template<typename MessageT>
  void publish(const MessageT & message) const
  {
    if (!model_ || model_->type() != std::type_index(typeid(MessageT))) {
      throw std::runtime_error("Publisher type mismatch");
    }
    model_->publish(&message);
  }

  template<typename MessageT>
  void publish(const std::shared_ptr<MessageT> & message) const
  {
    publish(*message);
  }

  size_t getNumSubscribers() const
  {
    return model_ ? model_->subscription_count() : 0;
  }

private:
  struct Base
  {
    virtual ~Base() = default;
    virtual std::type_index type() const = 0;
    virtual void publish(const void * message) = 0;
    virtual size_t subscription_count() const = 0;
  };

  template<typename MessageT>
  struct Model : Base
  {
    explicit Model(typename rclcpp::Publisher<MessageT>::SharedPtr publisher)
    : publisher(std::move(publisher)) {}

    ~Model() override
    {
      gestelt_ros2_compat::preserve_after_shutdown(publisher);
    }

    std::type_index type() const override {return std::type_index(typeid(MessageT));}
    void publish(const void * message) override
    {
      publisher->publish(*static_cast<const MessageT *>(message));
    }
    size_t subscription_count() const override
    {
      return publisher->get_subscription_count();
    }

    typename rclcpp::Publisher<MessageT>::SharedPtr publisher;
  };

  std::shared_ptr<Base> model_;
};

class Subscriber
{
public:
  Subscriber() = default;
  explicit Subscriber(rclcpp::SubscriptionBase::SharedPtr subscription)
  : subscription_(std::move(subscription)) {}
  ~Subscriber()
  {
    gestelt_ros2_compat::preserve_after_shutdown(subscription_);
  }

private:
  rclcpp::SubscriptionBase::SharedPtr subscription_;
};

class Timer
{
public:
  Timer() = default;
  explicit Timer(rclcpp::TimerBase::SharedPtr timer)
  : timer_(std::move(timer)) {}
  ~Timer()
  {
    gestelt_ros2_compat::preserve_after_shutdown(timer_);
  }
  void start()
  {
    if (timer_) {
      timer_->reset();
    }
  }
  void stop()
  {
    if (timer_) {
      timer_->cancel();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

class ServiceServer
{
public:
  ServiceServer() = default;
  explicit ServiceServer(rclcpp::ServiceBase::SharedPtr service)
  : service_(std::move(service)) {}
  ~ServiceServer()
  {
    gestelt_ros2_compat::preserve_after_shutdown(service_);
  }

private:
  rclcpp::ServiceBase::SharedPtr service_;
};

class ServiceClient
{
public:
  ServiceClient() = default;

  template<typename ServiceT>
  explicit ServiceClient(typename rclcpp::Client<ServiceT>::SharedPtr client)
  : model_(std::make_shared<Model<ServiceT>>(std::move(client))) {}

  template<typename ServiceT>
  static ServiceClient create(typename rclcpp::Client<ServiceT>::SharedPtr client)
  {
    ServiceClient result;
    result.model_ = std::make_shared<Model<ServiceT>>(std::move(client));
    return result;
  }

  template<typename Request, typename Response>
  bool call(Request & request, Response & response)
  {
    return model_ && model_->call(&request, &response);
  }

private:
  struct Base
  {
    virtual ~Base() = default;
    virtual bool call(void * request, void * response) = 0;
  };

  template<typename ServiceT>
  struct Model : Base
  {
    using Request = typename ServiceT::Request;
    using Response = typename ServiceT::Response;

    explicit Model(typename rclcpp::Client<ServiceT>::SharedPtr client)
    : client(std::move(client)) {}

    ~Model() override
    {
      gestelt_ros2_compat::preserve_after_shutdown(client);
    }

    bool call(void * request, void * response) override
    {
      if (!client || !client->wait_for_service(std::chrono::seconds(1))) {
        return false;
      }
      auto future = client->async_send_request(
        std::make_unique<Request>(*static_cast<Request *>(request)));
      if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        return false;
      }
      *static_cast<Response *>(response) = *future.get();
      return true;
    }

    typename rclcpp::Client<ServiceT>::SharedPtr client;
  };

  std::shared_ptr<Base> model_;
};

class NodeHandle
{
public:
  NodeHandle() = default;
  explicit NodeHandle(const std::string & name_space)
  : name_space_(name_space) {}

  template<typename T>
  bool param(const std::string & name, T & value, const T & default_value) const
  {
    auto node = node_ptr();
    const auto normalized_name = gestelt_ros2_compat::normalize_parameter_name(name);
    std::string parameter_name = name;
    if (!node->has_parameter(parameter_name)) {
      parameter_name = normalized_name;
    }
    if (!node->has_parameter(parameter_name)) {
      parameter_name = normalized_name;
      std::replace(parameter_name.begin(), parameter_name.end(), '.', '/');
    }
    if (!node->has_parameter(parameter_name)) {
      node->declare_parameter<T>(normalized_name, default_value);
      return node->get_parameter(normalized_name, value);
    }

    const auto parameter = node->get_parameter(parameter_name);
    if constexpr (std::is_same_v<T, double>) {
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<double>(parameter.as_int());
        return true;
      }
    } else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = static_cast<T>(parameter.as_double());
        return true;
      }
    }
    return node->get_parameter(parameter_name, value);
  }

  template<typename T>
  T param(const std::string & name, const T & default_value) const
  {
    T value{};
    param(name, value, default_value);
    return value;
  }

  bool getParam(const std::string & name, std::string & value) const
  {
    auto node = node_ptr();
    const auto normalized_name = gestelt_ros2_compat::normalize_parameter_name(name);
    std::string parameter_name = name;
    if (!node->has_parameter(parameter_name)) {
      parameter_name = normalized_name;
    }
    if (!node->has_parameter(parameter_name)) {
      parameter_name = normalized_name;
      std::replace(parameter_name.begin(), parameter_name.end(), '.', '/');
    }
    if (!node->has_parameter(parameter_name)) {
      return false;
    }
    return node->get_parameter(parameter_name, value);
  }

  template<typename MessageT>
  Publisher advertise(
    const std::string & topic, size_t queue_size = 10, bool latch = false) const
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size));
    if (latch) {
      qos.transient_local();
    }
    return Publisher::template create<MessageT>(
      node_ptr()->create_publisher<MessageT>(resolve_name(topic), qos));
  }

  template<typename MessageT, typename CallbackT>
  Subscriber subscribe(
    const std::string & topic, size_t queue_size, CallbackT && callback,
    const TransportHints & = TransportHints()) const
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = gestelt_ros2_compat::default_callback_group();
    return Subscriber(
      node_ptr()->create_subscription<MessageT>(
        resolve_name(topic),
        rclcpp::QoS(rclcpp::KeepLast(queue_size)).best_effort(),
        std::forward<CallbackT>(callback), options));
  }

  template<typename MessageT, typename ObjectT, typename MethodT>
  Subscriber subscribe(
    const std::string & topic, size_t queue_size, MethodT method, ObjectT * object,
    const TransportHints & = TransportHints()) const
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = gestelt_ros2_compat::default_callback_group();
    return Subscriber(
      node_ptr()->create_subscription<MessageT>(
        resolve_name(topic),
        rclcpp::QoS(rclcpp::KeepLast(queue_size)).best_effort(),
        std::bind(method, object, std::placeholders::_1), options));
  }

  template<typename MessageT, typename ObjectT>
  Subscriber subscribe(
    const std::string & topic, size_t queue_size,
    void (ObjectT::* method)(const std::shared_ptr<const MessageT> &),
    ObjectT * object, const TransportHints & = TransportHints()) const
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = gestelt_ros2_compat::default_callback_group();
    return Subscriber(
      node_ptr()->create_subscription<MessageT>(
        resolve_name(topic),
        rclcpp::QoS(rclcpp::KeepLast(queue_size)).best_effort(),
        [object, method](std::shared_ptr<const MessageT> message) {
          (object->*method)(message);
        }, options));
  }

  template<typename MessageT, typename ObjectT>
  Subscriber subscribe(
    const std::string & topic, size_t queue_size,
    void (ObjectT::* method)(std::shared_ptr<const MessageT>),
    ObjectT * object, const TransportHints & = TransportHints()) const
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = gestelt_ros2_compat::default_callback_group();
    return Subscriber(
      node_ptr()->create_subscription<MessageT>(
        resolve_name(topic),
        rclcpp::QoS(rclcpp::KeepLast(queue_size)).best_effort(),
        [object, method](std::shared_ptr<const MessageT> message) {
          (object->*method)(message);
        }, options));
  }

  template<typename ObjectT>
  Timer createTimer(
    const Duration & period, void (ObjectT::* callback)(TimerEvent &),
    ObjectT * object, bool = false, bool = false) const
  {
    return Timer(
      node_ptr()->create_wall_timer(
        period.get().to_chrono<std::chrono::nanoseconds>(),
        [object, callback]() {
          TimerEvent event;
          (object->*callback)(event);
        }, gestelt_ros2_compat::default_callback_group()));
  }

  template<typename ObjectT>
  Timer createTimer(
    const Duration & period, void (ObjectT::* callback)(const TimerEvent &),
    ObjectT * object, bool = false, bool = false) const
  {
    return Timer(
      node_ptr()->create_wall_timer(
        period.get().to_chrono<std::chrono::nanoseconds>(),
        [object, callback]() {
          const TimerEvent event;
          (object->*callback)(event);
        }, gestelt_ros2_compat::default_callback_group()));
  }

  template<typename ObjectT>
  Timer createTimer(
    const Duration & period, void (ObjectT::* callback)(), ObjectT * object,
    bool = false, bool = false) const
  {
    return Timer(
      node_ptr()->create_wall_timer(
        period.get().to_chrono<std::chrono::nanoseconds>(),
        [object, callback]() {(object->*callback)();},
        gestelt_ros2_compat::default_callback_group()));
  }

  template<typename ObjectT>
  ServiceServer advertiseService(
    const std::string & topic,
    bool (ObjectT::* callback)(
      mavros_msgs::srv::CommandBool::Request &,
      mavros_msgs::srv::CommandBool::Response &),
    ObjectT * object) const
  {
    return ServiceServer(
      node_ptr()->create_service<mavros_msgs::srv::CommandBool>(
        resolve_name(topic),
        [object, callback](
          const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request,
          std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response) {
          auto request_copy = *request;
          (object->*callback)(request_copy, *response);
        }, rmw_qos_profile_services_default,
        gestelt_ros2_compat::default_callback_group()));
  }

  template<typename ObjectT>
  ServiceServer advertiseService(
    const std::string & topic,
    bool (ObjectT::* callback)(
      mavros_msgs::srv::SetMode::Request &,
      mavros_msgs::srv::SetMode::Response &),
    ObjectT * object) const
  {
    return ServiceServer(
      node_ptr()->create_service<mavros_msgs::srv::SetMode>(
        resolve_name(topic),
        [object, callback](
          const std::shared_ptr<mavros_msgs::srv::SetMode::Request> request,
          std::shared_ptr<mavros_msgs::srv::SetMode::Response> response) {
          auto request_copy = *request;
          (object->*callback)(request_copy, *response);
        }, rmw_qos_profile_services_default,
        gestelt_ros2_compat::default_callback_group()));
  }

  template<typename ServiceT>
  ServiceClient serviceClient(const std::string & topic) const
  {
    return ServiceClient::template create<ServiceT>(
      node_ptr()->create_client<ServiceT>(resolve_name(topic)));
  }

  rclcpp::Node::SharedPtr node_ptr() const
  {
    if (!gestelt_ros2_compat::global_node()) {
      throw std::runtime_error("ros::init must be called before constructing NodeHandle");
    }
    return gestelt_ros2_compat::global_node();
  }

private:
  std::string resolve_name(const std::string & name) const
  {
    if (name.empty() || name.front() == '/' || name_space_.empty()) {
      return name;
    }
    if (name_space_ == "~") {
      return std::string(node_ptr()->get_name()) + "/" + name;
    }
    return name_space_ + "/" + name;
  }

  std::string name_space_;
};

class Rate
{
public:
  explicit Rate(double frequency)
  : rate_(frequency) {}
  bool sleep() {return rate_.sleep();}

private:
  rclcpp::Rate rate_;
};

class MultiThreadedSpinner
{
public:
  explicit MultiThreadedSpinner(size_t threads = 0)
  : thread_count_(threads == 0 ?
      std::max<size_t>(1, std::thread::hardware_concurrency()) : threads) {}
  void spin()
  {
    if (gestelt_ros2_compat::global_executor()) {
      gestelt_ros2_compat::configure_executor(thread_count_);
      gestelt_ros2_compat::executor_spinning() = true;
      gestelt_ros2_compat::global_executor()->spin();
      gestelt_ros2_compat::executor_spinning() = false;
      if (!rclcpp::ok()) {
        gestelt_ros2_compat::exit_after_shutdown();
      }
      gestelt_ros2_compat::global_executor().reset();
      gestelt_ros2_compat::executor_thread_count() = 0;
    }
  }

private:
  size_t thread_count_;
};

class AsyncSpinner : public MultiThreadedSpinner
{
public:
  explicit AsyncSpinner(size_t threads = 0)
  : MultiThreadedSpinner(threads) {}
  ~AsyncSpinner()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }
  void start()
  {
    if (!started_.exchange(true)) {
      worker_ = std::thread([this]() {spin();});
    }
  }

private:
  std::atomic<bool> started_{false};
  std::thread worker_;
};

inline void init(int argc, char ** argv, const std::string & name)
{
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  if (!gestelt_ros2_compat::global_node()) {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    gestelt_ros2_compat::global_node() =
      std::make_shared<rclcpp::Node>(name, options);
    gestelt_ros2_compat::default_callback_group() =
      gestelt_ros2_compat::global_node()->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    gestelt_ros2_compat::configure_executor(1);
  }
}

inline bool ok()
{
  return rclcpp::ok();
}

inline void spin()
{
  if (gestelt_ros2_compat::global_executor()) {
    gestelt_ros2_compat::configure_executor(1);
    gestelt_ros2_compat::executor_spinning() = true;
    gestelt_ros2_compat::global_executor()->spin();
    gestelt_ros2_compat::executor_spinning() = false;
    if (!rclcpp::ok()) {
      gestelt_ros2_compat::exit_after_shutdown();
    }
    gestelt_ros2_compat::global_executor().reset();
    gestelt_ros2_compat::executor_thread_count() = 0;
  }
}

inline void spinOnce()
{
  if (gestelt_ros2_compat::global_executor() &&
    !gestelt_ros2_compat::executor_spinning())
  {
    gestelt_ros2_compat::global_executor()->spin_some();
  }
}

inline void shutdown()
{
  rclcpp::shutdown();
  gestelt_ros2_compat::exit_after_shutdown();
}

inline void waitForShutdown()
{
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (gestelt_ros2_compat::global_executor()) {
    gestelt_ros2_compat::global_executor()->cancel();
  }
  gestelt_ros2_compat::exit_after_shutdown();
}

namespace topic
{
template<typename MessageT>
std::shared_ptr<MessageT> waitForMessage(
  const std::string & topic, const Duration & timeout = Duration(0.0))
{
  auto node = gestelt_ros2_compat::global_node();
  std::promise<std::shared_ptr<MessageT>> promise;
  auto future = promise.get_future();
  rclcpp::SubscriptionOptions options;
  options.callback_group = gestelt_ros2_compat::default_callback_group();
  auto subscription = node->create_subscription<MessageT>(
    topic, rclcpp::QoS(rclcpp::KeepLast(1)),
    [&promise](const std::shared_ptr<MessageT> message) {
      try {
        promise.set_value(message);
      } catch (const std::future_error &) {
      }
    }, options);

  const auto timeout_ns = timeout.toNSec() > 0 ? timeout.toNSec() : 10'000'000'000LL;
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() &&
    std::chrono::steady_clock::now() - start < std::chrono::nanoseconds(timeout_ns))
  {
    if (!gestelt_ros2_compat::executor_spinning()) {
      rclcpp::spin_some(node);
    }
    if (future.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
      return future.get();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw std::runtime_error("Timed out waiting for message on " + topic);
}
}  // namespace topic
}  // namespace ros

#define ROS_INFO(...) RCLCPP_INFO(gestelt_ros2_compat::logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(gestelt_ros2_compat::logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(gestelt_ros2_compat::logger(), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(gestelt_ros2_compat::logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(...) RCLCPP_INFO_STREAM(gestelt_ros2_compat::logger(), __VA_ARGS__)
#define ROS_WARN_STREAM(...) RCLCPP_WARN_STREAM(gestelt_ros2_compat::logger(), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(gestelt_ros2_compat::logger(), __VA_ARGS__)
#define ROS_FATAL_STREAM(...) RCLCPP_FATAL_STREAM(gestelt_ros2_compat::logger(), __VA_ARGS__)
#define ROS_INFO_THROTTLE(period, ...) \
  RCLCPP_INFO_THROTTLE(gestelt_ros2_compat::logger(), *gestelt_ros2_compat::global_node(), \
    static_cast<uint64_t>((period) * 1000.0), __VA_ARGS__)
#define ROS_WARN_THROTTLE(period, ...) \
  RCLCPP_WARN_THROTTLE(gestelt_ros2_compat::logger(), *gestelt_ros2_compat::global_node(), \
    static_cast<uint64_t>((period) * 1000.0), __VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) \
  RCLCPP_ERROR_THROTTLE(gestelt_ros2_compat::logger(), *gestelt_ros2_compat::global_node(), \
    static_cast<uint64_t>((period) * 1000.0), __VA_ARGS__)
#define ROS_FATAL_THROTTLE(period, ...) \
  RCLCPP_FATAL_THROTTLE(gestelt_ros2_compat::logger(), *gestelt_ros2_compat::global_node(), \
    static_cast<uint64_t>((period) * 1000.0), __VA_ARGS__)
#define ROS_INFO_STREAM_THROTTLE(period, ...) \
  RCLCPP_INFO_STREAM_THROTTLE(gestelt_ros2_compat::logger(), \
    *gestelt_ros2_compat::global_node(), \
    static_cast<uint64_t>((period) * 1000.0), __VA_ARGS__)

#define ROS_INFO_NAMED(node_name, ...) ROS_INFO(__VA_ARGS__)
#define ROS_WARN_NAMED(node_name, ...) ROS_WARN(__VA_ARGS__)
#define ROS_ERROR_NAMED(node_name, ...) ROS_ERROR(__VA_ARGS__)
#define ROS_FATAL_NAMED(node_name, ...) ROS_FATAL(__VA_ARGS__)
#define ROS_INFO_THROTTLE_NAMED(period, node_name, ...) ROS_INFO_THROTTLE(period, __VA_ARGS__)
#define ROS_WARN_THROTTLE_NAMED(period, node_name, ...) ROS_WARN_THROTTLE(period, __VA_ARGS__)
#define ROS_ERROR_THROTTLE_NAMED(period, node_name, ...) ROS_ERROR_THROTTLE(period, __VA_ARGS__)
#define ROS_FATAL_THROTTLE_NAMED(period, node_name, ...) ROS_FATAL_THROTTLE(period, __VA_ARGS__)

#endif  // GESTELT_ROS2_COMPAT_ROS_HPP_
