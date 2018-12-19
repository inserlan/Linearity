#pragma once

#include <vector>
#include <utility>

// �����
struct Point
{
	double x;
	double y;
};

// ����������Զ�ʱ������͹���εĵ�
struct PointConvex
{
	Point ptError;	// �Ŵ���ƫ������
	unsigned int nOriIndex;		// ��Ӧ��ԭʼ�������
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

	// �������Զ�
	// ���룺dbTheorySlope����б�ʣ� dbTheoryIntercept���۽ؾ�
	// ����ֵ���������Զȣ�%���ľ���ֵ
	double AbsoluteLinearity(double dbTheorySlope, double dbTheoryIntercept = 0) const;

	// �˻����Զ�
	// ����ֵ���˻����Զȣ�%���ľ���ֵ
	double TerminalBasedLinearity() const;

	// ƽ�ƶ˻����Զ�
	// ����ֵ��ƽ�ƶ˻����Զȣ�%���ľ���ֵ
	double TranslationTerminalBasedLinearity() const;

	// ������Զ�
	double ZeroBasedLinearity() const;

	// ǰ�˻����Զ�
	double FrontBasedLinearity() const;

	// �������Զ�
	double IndependentLinearity() const;

	// ��С����ֱ�����Զ�
	double LeastSquareLinearity() const;

private:
	// ��С���˷����ֱ��
	// ����ֵ��pair.firstб�ʣ� pair.second�ؾ�
	std::pair<double, double> LeastSquareFit() const;

	// ��ƫ����������ת����͹��������
	// ���룺dbTBSlope�˻�ֱ��б�ʣ�dbTBIntercept�˻�ֱ�߽ؾ࣬vtConvex�����͹��������
	void TranferErrorToConvex(double dbTBSlope, double dbTBIntercept, std::vector<PointConvex>& vtConvex) const;

	// ��ÿ��͹���ε�ĶԱ�
	void GetConvexOppositeSide(std::vector<PointConvex>& vtConvex, std::vector<OppositeSide>& vtOppositeSide) const;

private:
	std::vector<Point> m_vtPoint;
};

