#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
std::mutex m;
std::condition_variable cv;
std::string data;
bool ready = false;  // 条件
bool processed = false;  // 条件
void Worker() {
    std::unique_lock<std::mutex> lock1(m);

    // 等待主线程发送数据。
    cv.wait(lock1, [] { return ready; });

    // 等待后，继续拥有锁。
    std::cout << "工作线程正在处理数据..." << std::endl;
    // 睡眠一秒以模拟数据处理。
    std::this_thread::sleep_for(std::chrono::seconds(1));
    data += " 已处理";

    // 把数据发回主线程。
    processed = true;
    std::cout << "工作线程通知数据已经处理完毕。" << std::endl;

    // 通知前，手动解锁以防正在等待的线程被唤醒后又立即被阻塞。
    lock1.unlock();

    cv.notify_one();
}
int main() {
    std::thread worker(Worker);

    // 把数据发送给工作线程。
    {
        std::lock_guard<std::mutex> lock(m);
        std::cout << "主线程正在准备数据..." << std::endl;
        // 睡眠一秒以模拟数据准备。
        std::this_thread::sleep_for(std::chrono::seconds(1));
        data = "样本数据";
        ready = true;
        std::cout << "主线程通知数据已经准备完毕。" << std::endl;
    }
    cv.notify_one();

    // 等待工作线程处理数据。
    {
        std::unique_lock<std::mutex> lock(m);
        cv.wait(lock, [] { return processed; });
    }
    std::cout << "回到主线程，数据 = " << data << std::endl;

    worker.join();

    return 0;
}

