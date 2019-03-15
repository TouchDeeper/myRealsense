#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
using namespace std;
mutex m;
condition_variable cov;
bool ready = false;
bool processed = false;
atomic_bool alive{ true };
void showNum(int &f_, atomic_bool &alive_)
{
    while(alive_)
    {
        unique_lock<mutex> lk(m);
        cov.wait(lk,[]{ return ready || !alive;});
        if(!alive_)
            break;
        f_++;
        ready = false;
        processed= true;
//        lk.unlock();
        cout<<f_<<endl;
        cov.notify_one();
    }

}
int main() {
    vector<int> va;
    for (int i = 0; i < 10; ++i) {
        va.push_back(i);
    }
    int f = 0;



    std::thread t1(showNum,ref(f),ref(alive));
    auto sizeofVector = va.size();
    for (int j = 0; j < sizeofVector; ++j) {
        {
            lock_guard<mutex> lk0(m);
            f = va.back();
            cout<<f<<"    ";
            ready = true;
        }

        cov.notify_one();
        va.pop_back();
        {
            unique_lock<mutex> lk(m);
            cov.wait(lk,[]{return processed;});
            processed = false;
//            lk.unlock();
        }

    }
    alive = false;
    cov.notify_one();
    t1.join();
    return 0;
}
