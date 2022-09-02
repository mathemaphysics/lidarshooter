#include <vector>
#include <thread>
#include <future>
#include <numeric>
#include <iostream>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>
 
void accumulate(std::vector<int>::iterator first,
                std::vector<int>::iterator last,
                std::promise<int> accumulate_promise)
{
    int sum = std::accumulate(first, last, 0);
    accumulate_promise.set_value(sum);  // Notify future
}
 
void do_work(std::promise<void> barrier)
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    barrier.set_value();
}
 
int main()
{
    // Demonstrate using promise<int> to transmit a result between threads.
    std::vector<int> numbers = { 1, 2, 3, 4, 5, 6 };
    std::promise<int> accumulate_promise;
    std::future<int> accumulate_future = accumulate_promise.get_future();
    std::thread work_thread(accumulate, numbers.begin(), numbers.end(),
                            std::move(accumulate_promise));
 
    // future::get() will wait until the future has a valid result and retrieves it.
    // Calling wait() before get() is not needed
    //accumulate_future.wait();  // wait for result
    std::cout << "result=" << accumulate_future.get() << '\n';
    work_thread.join();  // wait for thread completion
 
    // Demonstrate using promise<void> to signal state between threads.
    std::promise<void> barrier;
    std::future<void> barrier_future = barrier.get_future();
    std::thread new_work_thread(do_work, std::move(barrier));
    barrier_future.wait();
    new_work_thread.join();

    // Make some vectors and shit
    Eigen::Vector3d x;
    x << 1, 2, 3;

    Eigen::AngleAxisf a(0.5, Eigen::Vector3f::UnitX());
    Eigen::Translation3f b(Eigen::Vector3f(0.1, -2.8, 7.0));
    Eigen::Affine3f trans = a * b * a.inverse();
    Eigen::AngleAxisf axis1;
    trans *= Eigen::Vector3f(1, 2, 3);
    Eigen::Affine3f something = a * Eigen::Translation3f(Eigen::Vector3f::Zero());
}
