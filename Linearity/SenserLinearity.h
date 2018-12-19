#pragma once

#include <vector>
#include <utility>

// 坐标点
struct Point
{
	double x;
	double y;
};

// 计算独立线性度时，描述凸边形的点
struct PointConvex
{
	Point ptError;	// 放大后的偏差坐标
	unsigned int nOriIndex;		// 对应的原始坐标序号
};

struct OppositeSide
{
	bool bExist;
	unsigned int index1;
	unsigned int index2;
};

class SenserLinearity
{
public:
	SenserLinearity(Point* pData, size_t nSize);
	~SenserLinearity();

	// 绝对线性度
	// 输入：dbTheorySlope理论斜率， dbTheoryIntercept理论截距
	// 返回值：绝对线性度（%）的绝对值
	double AbsoluteLinearity(double dbTheorySlope, double dbTheoryIntercept = 0) const;

	// 端基线性度
	// 返回值：端基线性度（%）的绝对值
	double TerminalBasedLinearity() const;

	// 平移端基线性度
	// 返回值：平移端基线性度（%）的绝对值
	double TranslationTerminalBasedLinearity() const;

	// 零基线性度
	double ZeroBasedLinearity() const;

	// 前端基线性度
	double FrontBasedLinearity() const;

	// 独立线性度
	double IndependentLinearity() const;

	// 最小二乘直线线性度
	double LeastSquareLinearity() const;

private:
	// 最小二乘法拟合直线
	// 返回值：pair.first斜率， pair.second截距
	std::pair<double, double> LeastSquareFit() const;

	// 将偏差坐标序列转换成凸边形序列
	// 输入：dbTBSlope端基直线斜率，dbTBIntercept端基直线截距，vtConvex输出的凸边形序列
	void TranferErrorToConvex(double dbTBSlope, double dbTBIntercept, std::vector<PointConvex>& vtConvex) const;

	// 求每个凸边形点的对边
	void GetConvexOppositeSide(std::vector<PointConvex>& vtConvex, std::vector<OppositeSide>& vtOppositeSide) const;

private:
	std::vector<Point> m_vtPoint;
};

