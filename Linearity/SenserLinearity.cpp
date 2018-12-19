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
	// �����ݷ���vector
	for (size_t i = 0; i < nSize; i++)
		m_vtPoint.push_back(pData[i]);
	// �����ݽ�������Xֵ��С����
	sort(m_vtPoint.begin(), m_vtPoint.end(), [](Point pt1, Point pt2) {
		return pt1.x < pt2.x;
	});
}


SenserLinearity::~SenserLinearity()
{
}

double SenserLinearity::AbsoluteLinearity(double dbTheorySlope, double dbTheoryIntercept) const
{
	// ��������������ֱ�ߵ�ƫ�����ֵ
	vector<double> vtError;
	for (auto& pt : m_vtPoint)
		vtError.push_back(fabs(pt.y - (dbTheorySlope * pt.x + dbTheoryIntercept)));

	// ������ƫ��
	double dbMaxError = *max_element(vtError.begin(), vtError.end());

	// �������Զ�
	return dbMaxError * 100 / (dbTheorySlope * (m_vtPoint.back().x - m_vtPoint.front().x));
}

double SenserLinearity::TerminalBasedLinearity() const
{
	// ���ݶ˵����ֱ��б�ʺͽؾ�
	double dbSlope = (m_vtPoint.back().y - m_vtPoint.front().y) / (m_vtPoint.back().x - m_vtPoint.front().x);
	double dbIntercept = m_vtPoint.front().y - dbSlope * m_vtPoint.front().x;
	// �������Զ�
	return AbsoluteLinearity(dbSlope, dbIntercept);
}

double SenserLinearity::TranslationTerminalBasedLinearity() const
{
	// ���ݶ˵����ֱ��б�ʺͽؾ�
	double dbSlope = (m_vtPoint.back().y - m_vtPoint.front().y) / (m_vtPoint.back().x - m_vtPoint.front().x);
	double dbIntercept = m_vtPoint.front().y - dbSlope * m_vtPoint.front().x;

	// ���������ڶ˵�ֱ�ߵ�ƫ��
	vector<double> vtError;
	for (auto& pt : m_vtPoint)
		vtError.push_back(pt.y - (dbSlope * pt.x + dbIntercept));

	// ������ƫ��
	double dbMaxError = *max_element(vtError.begin(), vtError.end());
	// ����С��ƫ��
	double dbMinError = *min_element(vtError.begin(), vtError.end());

	// ��ƽ�ƶ˻�ֱ�ߵĽؾ�
	double dbTranslationIntercept = dbIntercept + (dbMaxError + dbMinError) / 2;
	// �������Զ�
	return AbsoluteLinearity(dbSlope, dbTranslationIntercept);
}

double SenserLinearity::ZeroBasedLinearity() const
{
	// �������ԭ�������У�б�ʵ�������С�ĵ�
	// ��Ҫȥ��б��Ϊ�����ĵ㣬��x=0�ĵ�
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

	// ��ȡ�����Сб��
	double dbSlopeMin = (*iterMin).y / (*iterMin).x;
	double dbSlopeMax = (*iterMax).y / (*iterMax).x;

	// ��0.001�Ĳ����������б���µ����ƫ��
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

	// ��ȡ���ƫ����С��һ��
	auto iterMinError = min_element(vtMaxErrorPair.begin(), vtMaxErrorPair.end(), 
		[](pair<double, double> pa1, pair<double, double> pa2) {
		return fabs(pa1.second) < fabs(pa2.second);
	});

	// �������Զ�
	return AbsoluteLinearity((*iterMinError).first, 0);
}

double SenserLinearity::FrontBasedLinearity() const
{
	// �������ǰ�˵������У�б�ʵ�������С�ĵ�
	// ��Ҫȥ��б��Ϊ�����ĵ㣬��x=0�ĵ�
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

	// ��ȡ�����Сб��
	double dbSlopeMin = ((*iterMin).y - frontPt.y) / ((*iterMin).x - frontPt.x);
	double dbSlopeMax = ((*iterMax).y - frontPt.y) / ((*iterMax).x - frontPt.x);

	// ��0.001�Ĳ����������б���µ����ƫ��
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

	// ��ȡ���ƫ����С��һ��
	auto iterMinError = min_element(vtMaxErrorPair.begin(), vtMaxErrorPair.end(),
		[](pair<double, double> pa1, pair<double, double> pa2) {
		return fabs(pa1.second) < fabs(pa2.second);
	});

	// �������Զ�
	return AbsoluteLinearity((*iterMinError).first, frontPt.y - (*iterMinError).first * frontPt.x);
}

double SenserLinearity::IndependentLinearity() const
{
	// ͼ��Ӽ���
	// ���ݶ˵����ֱ��б�ʺͽؾ�
	double dbSlope = (m_vtPoint.back().y - m_vtPoint.front().y) / (m_vtPoint.back().x - m_vtPoint.front().x);
	double dbIntercept = m_vtPoint.front().y - dbSlope * m_vtPoint.front().x;

	// ��ȡ�Ŵ���ƫ������͹��������
	vector<PointConvex> vtConvex;
	TranferErrorToConvex(dbSlope, dbIntercept, vtConvex);

	// ��ÿ����ĶԱ�
	vector<OppositeSide> vtOppositeSide;
	GetConvexOppositeSide(vtConvex, vtOppositeSide);

	// ����㵽�ԱߵĴ��߳��ȣ�ǰ���Ǳ����жԱ�
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

	// Ѱ�Ҵ�����ĵ������
	auto iterMaxLen = max_element(vtVerticalLength.begin(), vtVerticalLength.end());
	auto indexMaxLen = iterMaxLen - vtVerticalLength.begin();

	// ��ȡ���㣬�ױ�ǰ�˵�͵ױߺ�˵������
	unsigned int nPeakIndex = static_cast<unsigned int>(indexMaxLen);
	unsigned int nBottomIndex1 = vtOppositeSide[nPeakIndex].index1;
	unsigned int nBottomIndex2 = vtOppositeSide[nPeakIndex].index2;

	// ������
	Point ptCenter1, ptCenter2;
	ptCenter1.x = (m_vtPoint[nPeakIndex].x + m_vtPoint[nBottomIndex1].x) / 2;
	ptCenter1.y = (m_vtPoint[nPeakIndex].y + m_vtPoint[nBottomIndex1].y) / 2;
	ptCenter2.x = (m_vtPoint[nPeakIndex].x + m_vtPoint[nBottomIndex2].x) / 2;
	ptCenter2.y = (m_vtPoint[nPeakIndex].y + m_vtPoint[nBottomIndex2].y) / 2;

	// ��������ֱ��
	double dbSlopeOpt = (ptCenter2.y - ptCenter1.y) / (ptCenter2.x - ptCenter1.x);
	double dbInterceptOpt = ptCenter1.y - dbSlopeOpt * ptCenter1.x;
	// �������Զ�
	return AbsoluteLinearity(dbSlopeOpt, dbInterceptOpt);
}

double SenserLinearity::LeastSquareLinearity() const
{
	// ���ֱ��
	pair<double, double> lineEle = LeastSquareFit();
	// �������Զ�
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
	// ���������ڶ˵�ֱ�ߵ�ƫ������еȱ����Ŵ󣬷Ŵ����Ӿ���������˴�Ϊ100��
	for (size_t i = 0; i < m_vtPoint.size(); i++)
		vtConvex.push_back(PointConvex{ m_vtPoint[i].x, (m_vtPoint[i].y - (dbTBSlope * m_vtPoint[i].x + dbTBIntercept)) * 100, i });

	// �������γ�͹����
	// ��Y>=0�ķ�һ�飬��X�������У���Y<0�ķ�һ�飬��X�������У�Ȼ��ϲ�����
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
	// �Ա�������������ǰ�㲻�ڸñ��ϣ���ǰ���x�ڶԱߵ�x�����
	for (int i = 0; i < int(vtConvex.size()); i++)
	{
		OppositeSide os{};
		// ������ǰ��֮��ı�
		for (int j = 0; j < (i - 1); j++)
		{
			// �жϵ�ǰ��xֵ�Ƿ��ڸñ�֮��
			if (((vtConvex[i].ptError.x <= vtConvex[j].ptError.x) && (vtConvex[i].ptError.x >= vtConvex[j + 1].ptError.x))
				|| ((vtConvex[i].ptError.x >= vtConvex[j].ptError.x) && (vtConvex[i].ptError.x <= vtConvex[j + 1].ptError.x)))
			{
				os.bExist = true;
				os.index1 = vtConvex[j].nOriIndex;
				os.index2 = vtConvex[j + 1].nOriIndex;
				break;
			}
		}
		// ֮ǰ��û��
		if (!os.bExist)
		{
			// ������ǰ��֮��ı�
			for (int j = (i + 1); j < (int(vtConvex.size()) - 1); j++)
			{
				// �жϵ�ǰ��xֵ�Ƿ��ڸñ�֮��
				if ((vtConvex[i].ptError.x <= vtConvex[j].ptError.x) && (vtConvex[i].ptError.x >= vtConvex[j + 1].ptError.x))
				{
					os.bExist = true;
					os.index1 = vtConvex[j].nOriIndex;
					os.index2 = vtConvex[j + 1].nOriIndex;
					break;
				}
			}
		}
		// �����û��
		if (!os.bExist)
		{
			// ���������β��
			if ((i != 0) && (i != (vtConvex.size() - 1)))
			{
				// �ж���β�������ı�
				// �жϵ�ǰ��xֵ�Ƿ��ڸñ�֮��
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
