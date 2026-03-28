#pragma once

// 线程耗时/CPU占用统计工具：
// - 适合在龙芯久久派（Linux）上直接打印每线程的 CPU% 与每周期工作耗时（不含 sleep）
// - 默认每 1s 输出一行
// - 通过 CMake 选项/宏 LQ_PROFILE_THREADS 控制开关

#include <cstdint>
#include <cstdio>
#include <time.h>

#if defined(__linux__)
#include <pthread.h>
#endif

#ifndef LQ_PROFILE_THREADS
#define LQ_PROFILE_THREADS 0
#endif

namespace lq {

inline uint64_t TimespecToNs(const timespec& ts)
{
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

inline uint64_t NowNs(clockid_t clockId)
{
    struct timespec ts;
    if (clock_gettime(clockId, &ts) != 0)
        return 0;
    return TimespecToNs(ts);
}

inline uint64_t NowWallNs()
{
    return NowNs(CLOCK_MONOTONIC);
}

inline uint64_t NowThreadCpuNs()
{
    return NowNs(CLOCK_THREAD_CPUTIME_ID);
}

inline void SetThisThreadName(const char* name)
{
#if defined(__linux__)
    if (!name || name[0] == '\0')
        return;
    // Linux 下线程名最长 16 字节（含 '\0'），超长会返回错误；忽略即可。
    pthread_setname_np(pthread_self(), name);
#else
    (void)name;
#endif
}

struct ThreadTimingReport {
    double wall_s = 0.0;
    double cpu_s = 0.0;
    double cpu_pct = 0.0;
    uint64_t iterations = 0;
    double work_avg_ms = 0.0;
    double work_max_ms = 0.0;
    uint64_t overruns = 0;
};

class ThreadTiming {
public:
    explicit ThreadTiming(const char* name, uint64_t reportIntervalNs = 1000000000ULL)
        : name_(name && name[0] ? name : "thread"),
          reportIntervalNs_(reportIntervalNs)
    {
#if LQ_PROFILE_THREADS
        lastReportWallNs_ = NowWallNs();
        lastReportCpuNs_ = NowThreadCpuNs();
#else
        (void)reportIntervalNs_;
#endif
    }

    const char* name() const { return name_; }

    void RecordWork(uint64_t workNs, bool overrun)
    {
#if LQ_PROFILE_THREADS
        ++iterations_;
        workSumNs_ += workNs;
        if (workNs > workMaxNs_)
            workMaxNs_ = workNs;
        if (overrun)
            ++overruns_;
#else
        (void)workNs;
        (void)overrun;
#endif
    }

    bool TryMakeReport(ThreadTimingReport* out)
    {
#if LQ_PROFILE_THREADS
        if (!out)
            return false;

        const uint64_t nowWall = NowWallNs();
        if (!nowWall)
            return false;

        const uint64_t wallDeltaNs = nowWall - lastReportWallNs_;
        if (wallDeltaNs < reportIntervalNs_)
            return false;

        const uint64_t nowCpu = NowThreadCpuNs();
        const uint64_t cpuDeltaNs = (nowCpu >= lastReportCpuNs_) ? (nowCpu - lastReportCpuNs_) : 0;

        ThreadTimingReport r;
        r.wall_s = static_cast<double>(wallDeltaNs) / 1e9;
        r.cpu_s = static_cast<double>(cpuDeltaNs) / 1e9;
        r.cpu_pct = wallDeltaNs ? (100.0 * static_cast<double>(cpuDeltaNs) / static_cast<double>(wallDeltaNs)) : 0.0;
        r.iterations = iterations_;
        if (iterations_)
        {
            r.work_avg_ms = (static_cast<double>(workSumNs_) / 1e6) / static_cast<double>(iterations_);
            r.work_max_ms = static_cast<double>(workMaxNs_) / 1e6;
        }
        r.overruns = overruns_;

        *out = r;

        lastReportWallNs_ = nowWall;
        lastReportCpuNs_ = nowCpu;
        iterations_ = 0;
        workSumNs_ = 0;
        workMaxNs_ = 0;
        overruns_ = 0;
        return true;
#else
        (void)out;
        return false;
#endif
    }

private:
    const char* name_;
    uint64_t reportIntervalNs_;

#if LQ_PROFILE_THREADS
    uint64_t lastReportWallNs_ = 0;
    uint64_t lastReportCpuNs_ = 0;
    uint64_t iterations_ = 0;
    uint64_t workSumNs_ = 0;
    uint64_t workMaxNs_ = 0;
    uint64_t overruns_ = 0;
#endif
};

inline void PrintThreadTimingLine(const char* name, const ThreadTimingReport& report, const char* extra = nullptr)
{
    if (!name)
        name = "thread";
    const double hz = (report.wall_s > 1e-9) ? (static_cast<double>(report.iterations) / report.wall_s) : 0.0;
    std::printf("[timing][%s] hz=%6.1f cpu=%5.1f%% iter=%6llu work_avg=%7.3fms work_max=%7.3fms overrun=%4llu%s%s\n",
                name,
                hz,
                report.cpu_pct,
                static_cast<unsigned long long>(report.iterations),
                report.work_avg_ms,
                report.work_max_ms,
                static_cast<unsigned long long>(report.overruns),
                (extra ? " " : ""),
                (extra ? extra : ""));
    std::fflush(stdout);
}

} // namespace lq
