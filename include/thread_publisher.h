#ifndef THREAD_PUBLISHER_H_
#define THREAD_PUBLISHER_H_

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>
#include <boost/bind.hpp>
#include <functional>


template <typename T>
class ThreadPublisher{
public:
  ThreadPublisher();
  ~ThreadPublisher();

  void Register(std::function<void(const std::shared_ptr<const T>&)> cb);
  void Start();
  void Publish(const std::shared_ptr<const T> msg);
  void Process();

  void ShutDown();

private:
  std::mutex msg_mutex;
  std::condition_variable msg_cond;
  std::queue<std::shared_ptr<const T> > msgs;

  std::thread publish_thread;
  std::vector<std::function<void(const std::shared_ptr<const T>&)>> callbacks;

  bool shutdown_requested;
};

// // publish function
// std::function<void(const FrameMatchMsg::ConstPtr&)> publish_feature_tracking_function = 
//     [&](const FrameMatchMsg::ConstPtr& feature_tracking_msg){
//   PublishFeatureTracking(feature_tracking_msg);
// };

#endif  // THREAD_PUBLISHER_H_