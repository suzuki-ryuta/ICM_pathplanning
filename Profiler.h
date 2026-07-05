#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

class ScopedProfiler
{
public:
	explicit ScopedProfiler(const char* name)
		:name_(name), start_(clock::now())
	{
	}

	~ScopedProfiler()
	{
		const auto end = clock::now();
		const double ms = std::chrono::duration<double, std::milli>(end - start_).count();
		auto& item = records()[name_];
		item.total_ms += ms;
		item.calls += 1;
	}

	ScopedProfiler(const ScopedProfiler&) = delete;
	ScopedProfiler& operator=(const ScopedProfiler&) = delete;

	static void reset()
	{
		records().clear();
	}

	static void add(const char* name, double ms, long long calls = 1)
	{
		auto& item = records()[name];
		item.total_ms += ms;
		item.calls += calls;
	}

	static void report(const std::string& title, double total_ms)
	{
		std::cout << "\n[PROFILE] " << title << "\n";
		std::cout << std::fixed << std::setprecision(3);
		std::cout << "[PROFILE] total: " << total_ms << " ms\n";
		for (const auto& entry : records()) {
			const double pct = total_ms > 0.0 ? 100.0 * entry.second.total_ms / total_ms : 0.0;
			std::cout << "[PROFILE] "
			          << std::setw(34) << std::left << entry.first
			          << " " << std::setw(10) << std::right << entry.second.total_ms << " ms"
			          << " " << std::setw(7) << pct << " %"
			          << " calls=" << entry.second.calls << "\n";
		}
	}

private:
	using clock = std::chrono::steady_clock;

	struct Record
	{
		double total_ms = 0.0;
		long long calls = 0;
	};

	static std::map<std::string, Record>& records()
	{
		static std::map<std::string, Record> instance;
		return instance;
	}

	const char* name_;
	clock::time_point start_;
};

#define PROFILE_CONCAT_IMPL(x, y) x##y
#define PROFILE_CONCAT(x, y) PROFILE_CONCAT_IMPL(x, y)
#define PROFILE_SCOPE(name) ScopedProfiler PROFILE_CONCAT(profile_scope_, __LINE__)(name)
