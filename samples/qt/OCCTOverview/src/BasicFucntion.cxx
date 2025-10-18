#include <vector>
#include <iostream>
#include <cmath>
#include "BasicFucntion.h"

// 三维点结构
struct Point3D {
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    // 点标量乘法
    Point3D operator*(double scalar) const {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }

    // 点加法
    Point3D operator+(const Point3D& other) const {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }
};

class BSplineEvaluator {
private:
    std::vector<Point3D> controlPoints;  // 控制点数组
    std::vector<double> knots;          // 节点向量
    int degree;                         // 曲线次数（degree）

public:
    // 构造函数
    BSplineEvaluator(const std::vector<Point3D>& points,
        const std::vector<double>& knotVector,
        int deg) {
        controlPoints = points;
        knots = knotVector;
        degree = deg;
    }

    // 查找参数u所在的节点区间索引[10](@ref)
    int FindSpan(double u) const {
        int n = controlPoints.size() - 1; // 最大控制点索引
        int m = knots.size() - 1;         // 最大节点索引

        // 特殊情况处理：u等于最后一个节点值
        if (u >= knots[m - degree]) {
            return m - degree - 1;
        }
        if (u <= knots[degree]) {
            return degree;
        }

        // 二分查找
        int low = degree;
        int high = m - degree;
        int mid = (low + high) / 2;

        while (u < knots[mid] || u >= knots[mid + 1]) {
            if (u < knots[mid]) {
                high = mid;
            }
            else {
                low = mid;
            }
            mid = (low + high) / 2;
        }

        return mid;
    }

    // 计算在参数u处的非零B样条基函数值[10](@ref)
    void BasisFunctions(int span, double u, std::vector<double>& basisValues) const {
        // 初始化存储空间
        std::vector<std::vector<double>> temp(degree + 1, std::vector<double>(degree + 1, 0.0));
        basisValues.resize(degree + 1, 0.0);

        // 首项设为1
        temp[0][0] = 1.0;

        // 递归计算基函数[9](@ref)
        for (int j = 1; j <= degree; ++j) {
            // 计算左侧项
            for (int r = 0; r < j; ++r) {
                double saved = 0.0;

                if (temp[r][j - 1] != 0.0) {
                    double term1 = u - knots[span + r - j + 1];
                    double term2 = knots[span + r + 1] - knots[span + r - j + 1];

                    if (term2 != 0.0) {
                        saved = term1 / term2 * temp[r][j - 1];
                    }
                }

                temp[r][j] = saved;
            }

            // 计算右侧项
            double saved = 0.0;
            for (int r = j - 1; r >= 0; --r) {
                double term1 = knots[span + r + 1] - u;
                double term2 = knots[span + r + 1] - knots[span + r - j + 1 + 1];

                if (term2 != 0.0) {
                    saved = term1 / term2 * temp[r][j - 1];
                }

                temp[r][j] += saved;
            }
        }

        // 提取最终结果（最后一列）
        for (int j = 0; j <= degree; ++j) {
            basisValues[j] = temp[j][degree];
        }
    }

    // 计算B样条曲线上参数u对应的点坐标[10](@ref)
    Point3D Evaluate(double u) const {
        // 1. 参数检查
        if (u < knots[degree] || u > knots[knots.size() - degree - 1]) {
            std::cerr << "警告：参数u=" << u << "超出有效范围 ["
                << knots[degree] << ", " << knots[knots.size() - degree - 1] << "]" << std::endl;
            return Point3D(0, 0, 0);
        }

        // 2. 查找节点区间
        int span = FindSpan(u);

        // 3. 计算非零基函数值
        std::vector<double> basisValues;
        BasisFunctions(span, u, basisValues);

        // 4. 计算曲线点坐标（加权求和）
        Point3D result(0, 0, 0);
        for (int i = 0; i <= degree; ++i) {
            int ctrlPointIndex = span - degree + i;
            double basis = basisValues[i];
            Point3D controlPoint = controlPoints[ctrlPointIndex];

            result = result + controlPoint * basis;
        }

        return result;
    }

    // 生成曲线上的一系列点（用于绘制）
    std::vector<Point3D> GenerateCurvePoints(int numSamples = 100) const {
        std::vector<Point3D> curvePoints;
        double uMin = knots[degree];
        double uMax = knots[knots.size() - degree - 1];
        double step = (uMax - uMin) / (numSamples - 1);

        for (int i = 0; i < numSamples; ++i) {
            double u = uMin + i * step;
            curvePoints.push_back(Evaluate(u));
        }

        return curvePoints;
    }
};

// 示例：生成均匀节点向量
std::vector<double> GenerateUniformKnots(int numCtrlPoints, int degree, double start = 0.0, double end = 1.0) {
    int totalKnots = numCtrlPoints + degree + 1;
    std::vector<double> knots(totalKnots);

    // 前degree+1个节点为start
    for (int i = 0; i <= degree; ++i) {
        knots[i] = start;
    }

    // 中间均匀分布的节点
    int internalPoints = totalKnots - 2 * (degree + 1);
    if (internalPoints > 0) {
        double step = (end - start) / (internalPoints + 1);
        for (int i = 1; i <= internalPoints; ++i) {
            knots[degree + i] = start + i * step;
        }
    }

    // 后degree+1个节点为end
    for (int i = totalKnots - degree - 1; i < totalKnots; ++i) {
        knots[i] = end;
    }

    return knots;
}

// 测试示例
int testBasizerFunction() {
    // 1. 定义控制点（三维）
    std::vector<Point3D> controlPoints = {
        {0.0, 0.0, 0.0},
        {1.0, 2.0, 1.0},
        {3.0, 1.0, 2.0},
        {5.0, 3.0, 1.0},
        {7.0, 0.0, 3.0}
    };

    // 2. 设置曲线次数
    int degree = 3; // 三次B样条曲线

    // 3. 生成节点向量
    auto knots = GenerateUniformKnots(controlPoints.size(), degree);

    // 4. 创建B样条求值器
    BSplineEvaluator evaluator(controlPoints, knots, degree);

    // 5. 测试单点计算
    double testU = 0.5;
    Point3D testPoint = evaluator.Evaluate(testU);

    std::cout << "B样条曲线在参数u=" << testU << "处的点坐标: ("
        << testPoint.x << ", " << testPoint.y << ", " << testPoint.z << ")" << std::endl;

    // 6. 生成完整曲线点集
    auto curvePoints = evaluator.GenerateCurvePoints(50);

    std::cout << "生成的曲线点数量: " << curvePoints.size() << std::endl;
    std::cout << "前5个点的坐标:" << std::endl;
    for (int i = 0; i < 5 && i < curvePoints.size(); ++i) {
        std::cout << "点" << i << ": (" << curvePoints[i].x << ", "
            << curvePoints[i].y << ", " << curvePoints[i].z << ")" << std::endl;
    }

    return 0;
}