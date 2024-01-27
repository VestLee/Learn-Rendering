#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace LRR
{
    namespace utils
    {
        /// @brief 这个类可以看作是一个线程池，但是只有一个线程，可以用来执行延迟或者定时任务，
        // 但内部没有队列，注意执行Dispatch之前确保没有正在执行的任务, 否则会丢弃新增的任务
        class ScheduledWorker
        {
            struct ScheduledTask
            {
                std::function<void()> task;
                unsigned int delay;
                unsigned int interval;
                bool precise;
            };

        public:
            ScheduledWorker()
            {
                m_worker_thread = std::thread(
                    [this]
                    {
                        while (true)
                        {
                            std::unique_lock<std::mutex> lock(m_mutex);
                            m_cv.wait(lock, [this]
                                      { return m_busy.load(); });
                            if (m_quit)
                                break;
                            if (m_scheduled_task.delay > 0)
                            {
                                m_cv.wait_for(lock, std::chrono::milliseconds(m_scheduled_task.delay));
                            }
                            if (!m_busy)
                                continue;
                            if (m_scheduled_task.interval > 0)
                            {
                                if (m_scheduled_task.precise)
                                {
                                    auto _interval = std::chrono::milliseconds(m_scheduled_task.interval);
                                    auto now = std::chrono::steady_clock::now();
                                    auto next = now + _interval;
                                    while (m_busy)
                                    {
                                        m_scheduled_task.task();
                                        if (!m_busy)
                                            break;
                                        m_cv.wait_until(lock, next);
                                        // wait_until has liitle delay
                                        // task may over run as well, so we need to adjust the next
                                        now = std::chrono::steady_clock::now();
                                        auto diff = now - next;
                                        if (diff <= _interval)
                                            next += _interval; // [[likely]]]
                                        else
                                            next = now + _interval;
                                    }
                                }
                                else
                                {
                                    while (m_busy)
                                    {
                                        m_scheduled_task.task();
                                        if (!m_busy)
                                            break;
                                        m_cv.wait_for(lock, std::chrono::milliseconds(m_scheduled_task.interval));
                                    }
                                }
                            }
                            if (m_scheduled_task.interval == 0 || m_do_on_stopped)
                            {
                                m_scheduled_task.task();
                            }
                            m_busy = false;
                        }
                    });
#if defined(__ANDROID__)
                pthread_setname_np(m_worker_thread.native_handle(), "ScheduledWorker");
#endif
            }
            ~ScheduledWorker()
            {
                {
                    std::unique_lock<std::mutex> lock(m_mutex);
                    m_busy = true;
                    m_do_on_stopped = false;
                    m_quit = true;
                }
                m_cv.notify_all();
                if (m_worker_thread.joinable())
                    m_worker_thread.join();
            }

            /// @brief 执行任务
            /// @param task 要执行的任务
            /// @param delay 延迟执行的时间，单位毫秒ms，如果为0，那么立即执行
            /// @param interval 定时任务的间隔，单位毫秒ms，如果为0，那么只延迟执行一次
            /// @param precise_schedule 是否精确调度，如果为true，那么每次调度都会计算时间差，否则这个间隔是在上一次任务执行完毕后开始计算
            void Dispatch(std::function<void()> task, unsigned int delay, unsigned int interval = 0, bool precise_schedule = false)
            {
                if (m_busy || !task)
                    return;
                {
                    std::unique_lock<std::mutex> lock(m_mutex);

                    m_scheduled_task.task = task;
                    m_scheduled_task.delay = delay;
                    m_scheduled_task.interval = interval;
                    m_scheduled_task.precise = precise_schedule;

                    m_busy = true;
                }
                m_cv.notify_all();
            }

            /// @brief 停止当前进行的任务
            /// @param doOnceOnStopped 是否在停止后执行一次任务，这个值对于定时任务才会生效，如果为true，那么会在停止时额外执行一次任务
            void Stop(bool doOnceOnStopped = false) noexcept
            {
                {
                    std::unique_lock<std::mutex> lock(m_mutex, std::defer_lock);
                    if (m_worker_thread.get_id() != std::this_thread::get_id())
                        lock.lock();
                    m_busy = false;
                    m_do_on_stopped = doOnceOnStopped;
                }
                m_cv.notify_all();
            }

            bool Busy() const noexcept
            {
                return m_busy;
            }

        private:
            std::thread m_worker_thread;
            std::atomic_bool m_busy = false;
            std::atomic_bool m_quit = false;
            ScheduledTask m_scheduled_task;
            bool m_do_on_stopped = false;
            std::condition_variable m_cv;
            std::mutex m_mutex;
        };
    }
}