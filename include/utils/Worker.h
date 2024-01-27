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
        /// @brief 这个类可以看作是一个线程池，但是只有一个线程
        // 内部没有队列，注意执行Dispatch之前确保没有正在执行的任务, 否则会丢弃新增的任务
        class Worker
        {
        public:
            Worker()
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
                            m_task();
                            m_busy = false;
                        }
                    });
#if defined(__ANDROID__)
                pthread_setname_np(m_worker_thread.native_handle(), "Worker");
#endif
            }
            ~Worker()
            {
                {
                    std::unique_lock<std::mutex> lock(m_mutex);
                    m_busy = true; // busy quitting :D
                    m_quit = true;
                }
                m_cv.notify_all();
                if (m_worker_thread.joinable())
                    m_worker_thread.join();
            }

            /// @brief 执行任务
            /// @param task 要执行的任务
            void Dispatch(std::function<void()> task) noexcept
            {
                if (m_busy || !task)
                    return;
                {
                    std::unique_lock<std::mutex> lock(m_mutex);

                    m_task = task;

                    m_busy = true;
                }
                m_cv.notify_all();
            }

            bool Busy() const noexcept
            {
                return m_busy;
            }

        private:
            std::function<void()> m_task;
            std::thread m_worker_thread;
            std::atomic_bool m_busy = false;
            std::atomic_bool m_quit = false;
            std::condition_variable m_cv;
            std::mutex m_mutex;
        };
    }
}