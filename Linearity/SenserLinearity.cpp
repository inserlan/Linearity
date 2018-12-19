#include "pch.h"
#include "SenserLinearity.h"
#include <assert.h>
#include <algorithm>
#include <numeric>
using namespace std;

SenserLinearity::SenserLinearity(Point* pData, size_t nSize)
{
	assert(pData != nullptr);
	assert(nSize != 0);
	// 将数据放入vector
	for (size_t i = 0; i < nSize; i++)
		m_vtPoint.push_back(pData[i]);
	// 对数据进行排序，X值从小到大
	sort(m_vtPoint.begin(), m_vtPoint.end(), [](Point pt1, Point pt2) {
		return pt1.x < pt2.x;
	});
}


SenserLinearity::~SenserLinearity()
{
}

double SenserLinearity::AbsoluteLinearity(double dbTheorySlope, double dbTheoryIntercept) const
{
	// 求各点相对于理论直线的偏差绝对值
	vector<double> vtError;
	for (auto& pt : m_vtPoint)
		vtError.push_back(fabs(pt.y - (dbTheorySlope * pt.x + dbTheoryIntercept)));

	// 求最大的偏差
	double dbMaxError = *max_element(vtError.begin(), vtError.end());

	// 返回线性度
	return dbMaxError * 100 / (dbTheorySlope * (m_vtPoint.back().x - m_vtPoint.front().x));
}

double SenserLinearity::TerminalBasedLinearity() const
{
	// 根据端点计算直线斜率和截距
	double dbSlope = (m_vtPoint.back().y - m_vtPoint.front().y) / (m_vtPoint.back().x - m_vtPoint.front().x);
	double dbIntercept = m_vtPoint.front().y - dbSlope * m_vtPoint.front().x;
	// 计算线性度
	return AbsoluteLinearity(dbSlope, dbIntercept);
}

double SenserLinearity::TranslationTerminalBasedLinearity() const
{
	// 根据端点计算直线斜率和截距
	double dbSlope = (m_vtPoint.back().y - m_vtPoint.front().y) / (m_vtPoint.back().x - m_vtPoint.front().x);
	double dbIntercept = m_vtPoint.front().y - dbSlope * m_vtPoint.front().x;

	// 求各点相对于端点直线的偏差
	vector<double> vtError;
	for (auto& pt : m_vtPoint)
		vtError.push_back(pt.y - (dbSlope * pt.x + dbIntercept));

	// 求最大的偏差
	double dbMaxError = *max_element(vtError.begin(), vtError.end());
	// 求最小的偏差
	double dbMinError = *min_element(vtError.begin(), vtError.end());

	// 求平移端基直线的截距
	double dbTranslationIntercept = dbIntercept + (dbMaxError + dbMinError) / 2;
	// 计算线性度
	return AbsoluteLinearity(dbSlope, dbTranslationIntercept);
}

double SenserLinearity::ZeroBasedLinearity() const
{
	// 求各点与原点连线中，斜率的最大和最小的点
	// 需要去掉斜率为无穷大的点，即x=0的点
	vector<Point> vtPointNoZero = m_vtPoint;
	vtPointNoZero.erase(remove_if(vtPointNoZero.begin(), vtPointNoZero.end(), [](Point pt) {
		return pt.x == 0;
	}), vtPointNoZero.end());
	auto iterMax = max_element(vtPointNoZero.begin(), vtPointNoZero.end(), [](Point pt1, Point pt2) {
		return (pt1.y / pt1.x) < (pt2.y / pt2.x);
	});
	auto iterMin = min_element(vtPointNoZero.begin(), vtPointNoZero.end(), [](Point pt1, Point pt2) {
		return (pt1.y / pt1.x) < (pt2.y / pt2.x);
	});

	// 获取最大最小斜率
	double dbSlopeMin = (*iterMin).y / (*iterMin).x;
	double dbSlopeMax = (*iterMax).y / (*iterMax).x;

	// 以0.001的步进，求各个斜率下的最大偏差
	vector<pair<double, double>> vtMaxErrorPair;
	for (double slp = dbSlopeMin; (slp - dbSlopeMax) <= 0.00001; slp += 0.001)
	{
		vector<double> vtError;
		for (auto& pt : m_vtPoint)
			vtError.push_back(pt.y - slp * pt.x);
		double dbMax = *max_element(vtError.begin(), vtError.end(), [](double v1, double v2) {
			return fabs(v1) < fabs(v2);
		});
		vtMaxErrorPair.push_back(pair<double, double>(slp, dbMax));
	}

	// 获取最大偏差最小的一组
	auto iterMinError = min_element(vtMaxErrorPair.begin(), vtMaxErrorPair.end(), 
		[](pair<double, double> pa1, pair<double, double> pa2) {
		return fabs(pa1.second) < fabs(pa2.second);
	});

	// 计算线性度
	return AbsoluteLinearity((*iterMinError).first, 0);
}

double SenserLinearity::FrontBasedLinearity() const
{
	// 求各点与前端点连线中，斜率的最大和最小的点
	// 需要去掉斜率为无穷大的点，即x=0的点
	Point frontPt = m_vtPoint.front();
	vector<Point> vtPointNoZero = m_vtPoint;
	vtPointNoZero.erase(remove_if(vtPointNoZero.begin(), vtPointNoZero.end(), [&](Point pt) {
		return (pt.x - frontPt.x) == 0;
	}), vtPointNoZero.end());
	auto iterMax = max_element(vtPointNoZero.begin(), vtPointNoZero.end(), [&](Point pt1, Point pt2) {
		return ((pt1.y - frontPt.y) / (pt1.x - frontPt.x)) < ((pt2.y - frontPt.y) / (pt2.x - frontPt.x));
	});
	auto iterMin = min_element(vtPointNoZero.begin(), vtPointNoZero.end(), [&](Point pt1, Point pt2) {
		return ((pt1.y - frontPt.y) / (pt1.x - frontPt.x)) < ((pt2.y - frontPt.y) / (pt2.x - frontPt.x));
	});

	// 获取最大最小斜率
	double dbSlopeMin = ((*iterMin).y - frontPt.y) / ((*iterMin).x - frontPt.x);
	double dbSlopeMax = ((*iterMax).y - frontPt.y) / ((*iterMax).x - frontPt.x);

	// 以0.001的步进，求各个斜率下的最大偏差
	vector<pair<double, double>> vtMaxErrorPair;
	for (double slp = dbSlopeMin; (slp - dbSlopeMax) <= 0.00001; slp += 0.001)
	{
		double dbIntercept = frontPt.y - slp * frontPt.x;
		vector<double> vtError;
		for (auto& pt : m_vtPoint)
			vtError.push_back(pt.y - (slp * pt.x + dbIntercept));
		double dbMax = *max_element(vtError.begin(), vtError.end(), [](double v1, double v2) {
			return fabs(v1) < fabs(v2);
		});
		vtMaxErrorPair.push_back(pair<double, double>(slp, dbMax));
	}

	// 获取最大偏差最小的一组
	auto iterMinError = min_element(vtMaxErrorPair.begin(), vtMaxErrorPair.end(),
		[](pair<double, double> pa1, pair<double, double> pa2) {
		return fabs(pa1.second) < fabs(pa2.second);
	});

	// 计算线性度
	return AbsoluteLinearity((*iterMinError).first, frontPt.y - (*iterMinError).first * frontPt.x);
}

double SenserLinearity::IndependentLinearity() const
{
	// 图解加计算
	// 根据端点计算直线斜率和截距
	double dbSlope = (m_vtPoint.back().y - m_vtPoint.front().y) / (m_vtPoint.back().x - m_vtPoint.front().x);
	double dbIntercept = m_vtPoint.front().y - dbSlope * m_vtPoint.front().x;

	// 获取放大后的偏差坐标凸边形序列
	vector<PointConvex> vtConvex;
	TranferErrorToConvex(dbSlope, dbIntercept, vtConvex);

	// 求每个点的对边
	vector<OppositeSide> vtOppositeSide;
	GetConvexOppositeSide(vtConvex, vtOppositeSide);

	// 求各点到对边的垂线长度，前提是必须有对边
	vector<double> vtVerticalLength;
	for (size_t i = 0; i < vtConvex.size(); i++)
	{
		if (vtOppositeSide[i].bExist)
		{
			double a = (vtConvex[vtOppositeSide[i].index2].ptError.y - vtConvex[vtOppositeSide[i].index1].ptError.y)
				/ (vtConvex[vtOppositeSide[i].index2].ptError.x - vtConvex[vtOppositeSide[i].index1].ptError.x);
			double b = vtConvex[vtOppositeSide[i].index1].ptError.y - a * vtConvex[vtOppositeSide[i].index1].ptError.x;
			vtVerticalLength.push_back(fabs(a * vtConvex[i].ptError.x + b - vtConvex[i].ptError.y));
		}
		else
		{
			vtVerticalLength.push_back(0);
		}
	}

	// 寻找垂线最长的点的索引
	auto iterMaxLen = max_element(vtVerticalLength.begin(), vtVerticalLength.end());
	auto indexMaxLen = iterMaxLen - vtVerticalLength.begin();

	// 获取顶点，底边前端点和底边后端点的索引
	unsigned int nPeakIndex = static_cast<unsigned int>(indexMaxLen);
	unsigned int nBottomIndex1 = vtOppositeSide[nPeakIndex].index1;
	unsigned int nBottomIndex2 = vtOppositeSide[nPeakIndex].index2;

	// 求重心
	Point ptCenter1, ptCenter2;
	ptCenter1.x = (m_vtPoint[nPeakIndex].x + m_vtPoint[nBottomIndex1].x) / 2;
	ptCenter1.y = (m_vtPoint[nPeakIndex].y + m_vtPoint[nBottomIndex1].y) / 2;
	ptCenter2.x = (m_vtPoint[nPeakIndex].x + m_vtPoint[nBottomIndex2].x) / 2;
	ptCenter2.y = (m_vtPoint[nPeakIndex].y + m_vtPoint[nBottomIndex2].y) / 2;

	// 计算最优直线
	double dbSlopeOpt = (ptCenter2.y - ptCenter1.y) / (ptCenter2.x - ptCenter1.x);
	double dbInterceptOpt = ptCenter1.y - dbSlopeOpt * ptCenter1.x;
	// 计算线性度
	return AbsoluteLinearity(dbSlopeOpt, dbInterceptOpt);
}

double SenserLinearity::LeastSquareLinearity() const
{
	// 拟合直线
	pair<double, double> lineEle = LeastSquareFit();
	// 计算线性度
	return AbsoluteLinearity(lineEle.first, lineEle.second);
}

pair<double, double> SenserLinearity::LeastSquareFit() const
{
	double avX = 0;
	double avY = 0;
	for (auto& pt : m_vtPoint)
	{
		avX += pt.x / m_vtPoint.size();
		avY += pt.y / m_vtPoint.size();
	}

	double dbXDis = 0;
	double dbYDis = 0;
	double dbXXSum = 0;
	double dbXYSum = 0;
	for (auto& pt : m_vtPoint)
	{
		dbXDis = pt.x - avX;
		dbYDis = pt.y - avY;
		dbXXSum += dbXDis * dbXDis;
		dbXYSum += dbXDis * dbYDis;
	}

	double dbSlope = dbXYSum / dbXXSum;
	double dbIntercept = avY - dbSlope * avX;
	return pair<double, double>(dbSlope, dbIntercept);
}

void SenserLinearity::TranferErrorToConvex(double dbTBSlope, double dbTBIntercept, std::vector<PointConvex>& vtConvex) const
{
	vtConvex.clear();
	// 求各点相对于端点直线的偏差，并进行等比例放大，放大倍数视具体情况，此处为100倍
	for (size_t i = 0; i < m_vtPoint.size(); i++)
		vtConvex.push_back(PointConvex{ m_vtPoint[i].x, (m_vtPoint[i].y - (dbTBSlope * m_vtPoint[i].x + dbTBIntercept)) * 100, i });

	// 将各点形成凸边形
	// 将Y>=0的放一组，按X增序排列，将Y<0的放一组，按X降序排列，然后合并两组
	auto iterSecondBegin = partition(vtConvex.begin(), vtConvex.end(), [](PointConvex pc) {
		return pc.ptError.y >= 0;
	});
	sort(vtConvex.begin(), iterSecondBegin, [](PointConvex pc1, PointConvex pc2) {
		return pc1.ptError.x < pc2.ptError.x;
	});
	sort(iterSecondBegin, vtConvex.end(), [](PointConvex pc1, PointConvex pc2) {
		return pc2.ptError.x < pc1.ptError.x;
	});
}

void SenserLinearity::GetConvexOppositeSide(vector<PointConvex>& vtConvex, vector<OppositeSide>& vtOppositeSide) const
{
	vtOppositeSide.clear();
	// 对边满足条件：当前点不在该边上，当前点的x在对边的x跨度内
	for (int i = 0; i < int(vtConvex.size()); i++)
	{
		OppositeSide os{};
		// 遍历当前点之后的边
		for (int j = 0; j < (i - 1); j++)
		{
			// 判断当前点x值是否在该边之间
			if (((vtConvex[i].ptError.x <= vtConvex[j].ptError.x) && (vtConvex[i].ptError.x >= vtConvex[j + 1].ptError.x))
				|| ((vtConvex[i].ptError.x >= vtConvex[j].ptError.x) && (vtConvex[i].ptError.x <= vtConvex[j + 1].ptError.x)))
			{
				os.bExist = true;
				os.index1 = vtConvex[j].nOriIndex;
				os.index2 = vtConvex[j + 1].nOriIndex;
				break;
			}
		}
		// 之前点没有
		if (!os.bExist)
		{
			// 遍历当前点之后的边
			for (int j = (i + 1); j < (int(vtConvex.size()) - 1); j++)
			{
				// 判断当前点x值是否在该边之间
				if ((vtConvex[i].ptError.x <= vtConvex[j].ptError.x) && (vtConvex[i].ptError.x >= vtConvex[j + 1].ptError.x))
				{
					os.bExist = true;
					os.index1 = vtConvex[j].nOriIndex;
					os.index2 = vtConvex[j + 1].nOriIndex;
					break;
				}
			}
		}
		// 如果还没有
		if (!os.bExist)
		{
			// 如果不是首尾点
			if ((i != 0) && (i != (vtConvex.size() - 1)))
			{
				// 判断首尾点相连的边
				// 判断当前点x值是否在该边之间
				if (((vtConvex[i].ptError.x <= vtConvex.front().ptError.x) && (vtConvex[i].ptError.x >= vtConvex.back().ptError.x))
					|| ((vtConvex[i].ptError.x >= vtConvex.front().ptError.x) && (vtConvex[i].ptError.x <= vtConvex.back().ptError.x)))
				{
					os.bExist = true;
					os.index1 = vtConvex.front().nOriIndex;
					os.index2 = vtConvex.back().nOriIndex;
				}
			}
		}
		vtOppositeSide.push_back(os);
	}
}
