#pragma once
#include <string>
#include <chrono>
#include <iostream>

struct scoped_benchmark
{
	scoped_benchmark(const std::string& title)
	{
		std::cout << title << "...";
		start = std::chrono::high_resolution_clock::now();    
	}

	~scoped_benchmark()
	{		
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
		std::cout << " [" << duration.count() << "us]" << std::endl;
	}

private:
	std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

template<
	typename F, 
	typename Result = std::result_of_t<F()>,
	typename std::enable_if<!std::is_same<Result, void>::value, int>::type = 0
>
Result benchmark(const std::string& title, F&& f)
{
	scoped_benchmark sb{title};
	return f();
}

template<
	typename F, 
	typename Result = std::result_of_t<F()>,
	typename std::enable_if<std::is_same<Result, void>::value, int>::type = 0
>
Result benchmark(const std::string& title, F&& f)
{
	scoped_benchmark sb{title};
	f();
}