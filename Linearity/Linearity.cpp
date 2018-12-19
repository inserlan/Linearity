// Linearity.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "SenserLinearity.h"
#include <random>

int main()
{
	// 论文上的例子
	 /*Point ptArray[6] = { {0, 0.1}, {1, 1.0}, {2,1.8}, {3,2.6}, {4,3.0}, {5,3.8} };
	 SenserLinearity sl(ptArray, 6);
	 std::cout << "绝对线性度：±" << sl.AbsoluteLinearity(0.8) << "%" << std::endl;*/

	 // 正态分布的例子
	Point ptArray[10];
	std::random_device rd{};
	std::mt19937 gen{ rd() };
	std::normal_distribution<> d{ 0, 0.3 };
	for (size_t i = 0; i < 10; i++)
	{
		ptArray[i].x = i;
		ptArray[i].y = 1.2 * i + 0.8 + d(gen);
	}
	SenserLinearity sl(ptArray, 10);
	std::cout << "绝对线性度：±" << sl.AbsoluteLinearity(1.2, 0.8) << "%" << std::endl;

	std::cout << "端基线性度：±" << sl.TerminalBasedLinearity() << "%" << std::endl;
	std::cout << "平移端基线性度：±" << sl.TranslationTerminalBasedLinearity() << "%" << std::endl;
	std::cout << "零基线性度：±" << sl.ZeroBasedLinearity() << "%" << std::endl;
	std::cout << "前端基基线性度：±" << sl.FrontBasedLinearity() << "%" << std::endl;
	std::cout << "独立线性度：±" << sl.IndependentLinearity() << "%" << std::endl;
	std::cout << "最小二乘法线性度：±" << sl.LeastSquareLinearity() << "%" << std::endl;
}