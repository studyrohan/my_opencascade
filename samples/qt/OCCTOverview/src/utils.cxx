#include <Geom_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColGeom_HArray1OfCurve.hxx>
#include <GeomFill_AppSurf.hxx>
#include <Precision.hxx>
#include <Standard_Real.hxx>
#include <iostream>
#include <TopoDS_Edge.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <iostream>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Hyperbola.hxx>
#include <Geom_Parabola.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_OffsetCurve.hxx>
#include "utils.h"
#include "Geom_Plane.hxx"
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <Geom_Axis1Placement.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom2dAPI_PointsToBSpline.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include "gp_Cylinder.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepBuilderAPI_MakeFace.hxx"
#include "gp_Torus.hxx"
#include "BRepBuilderAPI_MakeShell.hxx"
#include "BRepBuilderAPI_MakeSolid.hxx"

#include <Geom_BSplineSurface.hxx> // 样条曲面头文件
#include <TColgp_Array2OfPnt.hxx>   // 控制点数组头文件
#include <BRepTools.hxx>
#include <iostream>
#include "BRepPrimAPI_MakeCylinder.hxx"

void PrintBSplineSurfacePoles(const Handle(Geom_BSplineSurface)& surface)
{
	// 获取控制点数组
	const TColgp_Array2OfPnt& poles = surface->Poles();

	// 获取行和列的索引范围
	Standard_Integer rowLower = poles.LowerRow();
	Standard_Integer rowUpper = poles.UpperRow();
	Standard_Integer colLower = poles.LowerCol();
	Standard_Integer colUpper = poles.UpperCol();

	std::cout << "B-Spline Surface Control Points ("
		<< (rowUpper - rowLower + 1) << " rows, "
		<< (colUpper - colLower + 1) << " columns):" << std::endl;

	// 遍历所有控制点
	for (Standard_Integer i = rowLower; i <= rowUpper; ++i)
	{
		for (Standard_Integer j = colLower; j <= colUpper; ++j)
		{
			// 获取当前控制点
			gp_Pnt point = poles(i, j);

			// 打印控制点索引和坐标
			std::cout << "  Pole[" << i << "][" << j << "] = ("
				<< point.X() << ", " << point.Y() << ", " << point.Z() << ")" << std::endl;
		}
	}
}

void GetFaceSurfaceInfo(const TopoDS_Face& face) {
	// 1. 获取面的几何曲面和位置信息
	TopLoc_Location location;
	Handle(Geom_Surface) surface = BRep_Tool::Surface(face, location);

	if (surface.IsNull()) {
		std::cerr << "Error: Failed to get surface from face." << std::endl;
		return;
	}

	// 2. 识别并处理具体的曲面类型
	if (surface->IsKind(STANDARD_TYPE(Geom_Plane))) {
		// 处理平面
		Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surface);
		if (!plane.IsNull()) {
			gp_Ax1 axis = plane->Axis();
			gp_Pnt origin = axis.Location(); // 平面原点
			gp_Dir normal = axis.Direction(); // 平面法向
			std::cout << "Surface Type: Plane" << std::endl;
			std::cout << "  - Origin: (" << origin.X() << ", " << origin.Y() << ", " << origin.Z() << ")" << std::endl;
			std::cout << "  - Normal: (" << normal.X() << ", " << normal.Y() << ", " << normal.Z() << ")" << std::endl;
		}
	}
	else if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) {
		// 处理NURBS曲面
		Handle(Geom_BSplineSurface) bspline = Handle(Geom_BSplineSurface)::DownCast(surface);
		if (!bspline.IsNull()) {
			int uDegree = bspline->UDegree();
			int vDegree = bspline->VDegree();
			int numUPoles = bspline->NbUPoles();
			int numVPoles = bspline->NbVPoles();
			TColgp_Array2OfPnt poles = bspline->Poles();
			PrintBSplineSurfacePoles(bspline);

			std::cout << "Surface Type: BSplineSurface" << std::endl;
			std::cout << "  - U Degree: " << uDegree << ", V Degree: " << vDegree << std::endl;
			std::cout << "  - Number of U Poles: " << numUPoles << ", V Poles: " << numVPoles << std::endl;
			// 可进一步获取控制点、节点向量等信息[5](@ref)
		}
	}
	else if (surface->IsKind(STANDARD_TYPE(Geom_CylindricalSurface))) {
		// 处理圆柱面
		Handle(Geom_CylindricalSurface) cylinder = Handle(Geom_CylindricalSurface)::DownCast(surface);
		if (!cylinder.IsNull()) {
			gp_Cylinder cyl = cylinder->Cylinder();
			gp_Ax1 axis = cyl.Axis();
			double radius = cyl.Radius();
			std::cout << "Surface Type: CylindricalSurface" << std::endl;
			std::cout << "  - Radius: " << radius << std::endl;
			std::cout << "  - Axis Location: (" << axis.Location().X() << ", " << axis.Location().Y() << ", " << axis.Location().Z() << ")" << std::endl;
			std::cout << "  - Axis Direction: (" << axis.Direction().X() << ", " << axis.Direction().Y() << ", " << axis.Direction().Z() << ")" << std::endl;
		}
	}
	// 可以继续添加其他曲面类型的处理逻辑，如球面、锥面等...

	// 3. 注意：从BRep_Tool::Surface获取的曲面可能包含位置变换信息[1](@ref)
	if (!location.IsIdentity()) {
		// 如果需要将曲面上的点转换到全局坐标系，需应用location的变换
		// 例如：gp_Pnt globalPoint = location.Transformation().Transformed(localPoint);
		std::cout << "Note: The surface has a non-identity location." << std::endl;
	}
}



void PrintGeneralCurveInfo(const Handle(Geom2d_Curve)& theCurve, const Standard_Real theParam ) {
	if (theCurve.IsNull()) {
		std::cout << "Error: Curve is null." << std::endl;
		return;
	}

	// 1. 输出曲线动态类型名称
	Standard_CString typeName = theCurve->DynamicType()->Name();
	std::cout << "Curve Type: " << typeName << std::endl;

	// 2. 输出通用曲线信息
	std::cout << "  - First Parameter: " << theCurve->FirstParameter() << std::endl;
	std::cout << "  - Last Parameter: " << theCurve->LastParameter() << std::endl;
	std::cout << "  - Is Closed: " << (theCurve->IsClosed() ? "True" : "False") << std::endl;
	std::cout << "  - Is Periodic: " << (theCurve->IsPeriodic() ? "True" : "False") << std::endl;
	if (theCurve->IsPeriodic()) {
		std::cout << "  - Period: " << theCurve->Period() << std::endl;
	}

	// 3. 计算并输出指定参数处的点和导数 (示例)
	Standard_Real u = theCurve->FirstParameter() + theParam * (theCurve->LastParameter() - theCurve->FirstParameter());
	gp_Pnt2d P;
	theCurve->D0(u, P);
	std::cout << "  - Point at parameter " << u << ": (" << P.X() << ", " << P.Y() << ")" << std::endl;

	// 4. 根据曲线具体类型向下转换并输出特定信息
	if (theCurve->IsKind(STANDARD_TYPE(Geom2d_Line))) {
		Handle(Geom2d_Line) aLine = Handle(Geom2d_Line)::DownCast(theCurve);
		gp_Lin2d lin = aLine->Lin2d();
		std::cout << "  - Line Location: (" << lin.Location().X() << ", " << lin.Location().Y() << ")" << std::endl;
		std::cout << "  - Line Direction: (" << lin.Direction().X() << ", " << lin.Direction().Y() << ")" << std::endl;
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom2d_Circle))) {
		Handle(Geom2d_Circle) aCircle = Handle(Geom2d_Circle)::DownCast(theCurve);
		gp_Circ2d circ = aCircle->Circ2d();
		std::cout << "  - Circle Location (Center): (" << circ.Location().X() << ", " << circ.Location().Y() << ")" << std::endl;
		std::cout << "  - Circle Radius: " << circ.Radius() << std::endl;
		// 还可获取 XAxis 和 YAxis 的方向
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom2d_BSplineCurve))) {
		Handle(Geom2d_BSplineCurve) aBSpline = Handle(Geom2d_BSplineCurve)::DownCast(theCurve);
		std::cout << "  - BSpline Degree: " << aBSpline->Degree() << std::endl;
		std::cout << "  - Number of Poles: " << aBSpline->NbPoles() << std::endl;
		std::cout << "  - Number of Knots: " << aBSpline->NbKnots() << std::endl;
		// 可进一步遍历控制点、节点、权重等
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom2d_TrimmedCurve))) {
		Handle(Geom2d_TrimmedCurve) aTrimmed = Handle(Geom2d_TrimmedCurve)::DownCast(theCurve);
		Handle(Geom2d_Curve) basisCurve = aTrimmed->BasisCurve();
		std::cout << "  - Basis Curve Type: " << basisCurve->DynamicType()->Name() << std::endl;
		std::cout << "  - Trimmed Parameters: [" << aTrimmed->FirstParameter() << ", " << aTrimmed->LastParameter() << "]" << std::endl;
		// 可以递归调用 PrintGeneralCurveInfo(basisCurve) 来输出基准曲线的信息
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom2d_OffsetCurve))) {
		Handle(Geom2d_OffsetCurve) anOffset = Handle(Geom2d_OffsetCurve)::DownCast(theCurve);
		std::cout << "  - Offset Distance: " << anOffset->Offset() << std::endl;
		Handle(Geom2d_Curve) basisCurve = anOffset->BasisCurve();
		std::cout << "  - Basis Curve Type: " << basisCurve->DynamicType()->Name() << std::endl;
		// 可以递归调用 PrintGeneralCurveInfo(basisCurve) 来输出基准曲线的信息
	}
	// ... 可以继续添加其他曲线类型的处理逻辑，如椭圆、抛物线、Bezier曲线等。
	std::cout << std::endl;
}


void GetCurveInfoFromEdge(const TopoDS_Edge& theEdge) {
	// 声明变量用于存储曲线在边上的参数范围
	Standard_Real firstParam, lastParam;

	// 从拓扑边获取几何曲线的句柄及其参数范围
	Handle(Geom_Curve) curve = BRep_Tool::Curve(theEdge, firstParam, lastParam);

	if (curve.IsNull()) {
		std::cout << "Error: Failed to extract geometric curve from the edge." << std::endl;
		return;
	}

	// 现在可以使用 curve 对象，调用上述表格中的方法获取其具体信息
	std::cout << "Curve Type: " << curve->DynamicType()->Name() << std::endl;
	std::cout << "Parameter Range on this Edge: [" << firstParam << ", " << lastParam << "]" << std::endl;
	// ... 进一步根据曲线具体类型查询信息
}

// 简化版的Cox-de Boor递归计算示例（实际使用需考虑边界和效率）
double BasisFunction(int i, int p, double u, const TColStd_Array1OfReal& knots) {
	if (p == 0) {
		if (knots(i) <= u && u < knots(i + 1)) return 1.0;
		else return 0.0;
	}
	else {
		double left = (knots(i + p) - knots(i)) != 0 ? (u - knots(i)) / (knots(i + p) - knots(i)) * BasisFunction(i, p - 1, u, knots) : 0;
		double right = (knots(i + p + 1) - knots(i + 1)) != 0 ? (knots(i + p + 1) - u) / (knots(i + p + 1) - knots(i + 1)) * BasisFunction(i + 1, p - 1, u, knots) : 0;
		return left + right;
	}
}

/**
 * 在节点向量中查找参数u所在的节点区间
 * @param n 控制点数量-1（最大控制点索引）
 * @param p 曲线次数（degree）
 * @param u 要查找的参数值
 * @param U 节点向量数组（长度为n+p+2）
 * @param numKnots 节点向量总长度
 * @return 参数u所在的节点区间下标
 */
//int FindSpan(int n, int p, double u, const double* U, int numKnots) {
int FindSpan(int n, int p, double u, const TColStd_Array1OfReal & knots) {
	// 特殊情况处理：如果u等于最后一个节点值，直接返回n
	if (u == knots[n + 1]) {
		return n;
	}

	// 二分查找初始化
	int low = p;
	int high = n + 1;
	int mid = (low + high) / 2;

	// 二分查找核心循环
	while (u < knots[mid] || u >= knots[mid + 1]) {
		if (u < knots[mid]) {
			high = mid;      // 在左半部分继续查找
		}
		else {
			low = mid;       // 在右半部分继续查找
		}
		mid = (low + high) / 2;
	}

	return mid;
}

// 然后遍历所有控制点进行加权求和
gp_Pnt ComputePointOnCurve(double u, const TColgp_Array1OfPnt& poles, int degree, const TColStd_Array1OfReal& knots) {
	int numPoles = poles.Length();
	int order = degree + 1;
	gp_Pnt result(0, 0, 0);
	double totalWeight = 0.0;

	// 找到u所在的节点区间
	int span = FindSpan(numPoles-1, degree, u, knots); // 需要实现FindSpan函数

	// 计算非零的基函数值
	for (int i = 0; i <= degree; i++) {
		int idx = span - degree + i;
		double basisVal = BasisFunction(idx, degree, u, knots); // 计算基函数
		gp_Pnt pole = poles(idx + 1); // 注意索引，假设poles索引从1开始
		result.SetX(result.X() + basisVal * pole.X());
		result.SetY(result.Y() + basisVal * pole.Y());
		result.SetZ(result.Z() + basisVal * pole.Z());
	}
	return result;
}

void PrintGeomCurveInfo(const Handle(Geom_Curve)& theCurve, const Standard_Real theParamSample) {
	if (theCurve.IsNull()) {
		std::cout << "Error: Curve is null." << std::endl;
		return;
	}

	// 1. 输出曲线的基本类型和全局信息
	Standard_CString typeName = theCurve->DynamicType()->Name();
	std::cout << "=== Curve Type: " << typeName << " ===" << std::endl;
	std::cout << "  - First Parameter: " << theCurve->FirstParameter() << std::endl;
	std::cout << "  - Last Parameter: " << theCurve->LastParameter() << std::endl;
	std::cout << "  - Is Closed: " << (theCurve->IsClosed() ? "True" : "False") << std::endl;
	std::cout << "  - Is Periodic: " << (theCurve->IsPeriodic() ? "True" : "False") << std::endl;

	// 2. 计算并输出一个样本点（参数中点）的信息
	Standard_Real uSample = theCurve->FirstParameter() + theParamSample * (theCurve->LastParameter() - theCurve->FirstParameter());
	gp_Pnt P;
	theCurve->D0(uSample, P);
	std::cout << "  - Sample Point at parameter " << uSample << ": (" << P.X() << ", " << P.Y() << ", " << P.Z() << ")" << std::endl;

	// 3. 根据具体的曲线类型，向下转换并输出特定信息
	if (theCurve->IsKind(STANDARD_TYPE(Geom_Line))) {
		Handle(Geom_Line) aLine = Handle(Geom_Line)::DownCast(theCurve);
		gp_Lin lin = aLine->Lin();
		gp_Pnt loc = lin.Location();
		gp_Dir dir = lin.Direction();
		std::cout << "  - Line Location (Point on line): (" << loc.X() << ", " << loc.Y() << ", " << loc.Z() << ")" << std::endl;
		std::cout << "  - Line Direction: (" << dir.X() << ", " << dir.Y() << ", " << dir.Z() << ")" << std::endl;
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom_Circle))) {
		Handle(Geom_Circle) aCircle = Handle(Geom_Circle)::DownCast(theCurve);
		gp_Circ circ = aCircle->Circ();
		gp_Pnt loc = circ.Location();
		std::cout << "  - Circle Center: (" << loc.X() << ", " << loc.Y() << ", " << loc.Z() << ")" << std::endl;
		std::cout << "  - Circle Radius: " << circ.Radius() << std::endl;
		std::cout << "  - Circle Axis Direction: (" << circ.Axis().Direction().X() << ", "
			<< circ.Axis().Direction().Y() << ", " << circ.Axis().Direction().Z() << ")" << std::endl;
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom_Ellipse))) {
		Handle(Geom_Ellipse) anEllipse = Handle(Geom_Ellipse)::DownCast(theCurve);
		gp_Elips elips = anEllipse->Elips();
		gp_Pnt loc = elips.Location();
		std::cout << "  - Ellipse Center: (" << loc.X() << ", " << loc.Y() << ", " << loc.Z() << ")" << std::endl;
		std::cout << "  - Major Radius: " << elips.MajorRadius() << std::endl;
		std::cout << "  - Minor Radius: " << elips.MinorRadius() << std::endl;
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom_Parabola))) {
		Handle(Geom_Parabola) aParabola = Handle(Geom_Parabola)::DownCast(theCurve);
		gp_Parab para = aParabola->Parab();
		gp_Pnt foc = para.Focus();
		std::cout << "  - Parabola Focus: (" << foc.X() << ", " << foc.Y() << ", " << foc.Z() << ")" << std::endl;
		std::cout << "  - Focal Length: " << para.Focal() << std::endl;
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom_Hyperbola))) {
		Handle(Geom_Hyperbola) aHyperbola = Handle(Geom_Hyperbola)::DownCast(theCurve);
		gp_Hypr hypr = aHyperbola->Hypr();
		gp_Pnt loc = hypr.Location();
		std::cout << "  - Hyperbola Center: (" << loc.X() << ", " << loc.Y() << ", " << loc.Z() << ")" << std::endl;
		std::cout << "  - Major Radius: " << hypr.MajorRadius() << std::endl;
		std::cout << "  - Minor Radius: " << hypr.MinorRadius() << std::endl;
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom_BSplineCurve))) {
		Handle(Geom_BSplineCurve) aBSpline = Handle(Geom_BSplineCurve)::DownCast(theCurve);
		std::cout << "  - BSpline Degree: " << aBSpline->Degree() << std::endl;
		std::cout << "  - Number of Poles: " << aBSpline->NbPoles() << std::endl;
		TColgp_Array1OfPnt poles = aBSpline->Poles();
		Standard_Integer lowerIndex = poles.Lower();
		Standard_Integer upperIndex = poles.Upper();
		std::cout << "  - Point messages: ";
		for (Standard_Integer i = lowerIndex; i <= upperIndex; ++i) {
			// 使用 Value(i) 方法获取索引为 i 的点
			gp_Pnt point = poles.Value(i);
			std::cout << "point" << i << ",(" << point.X() << " " << point.Y() << " " << point.Z() << ") ";
		}
		std::cout << std::endl;
		std::cout << "  - Number of Knots: " << aBSpline->NbKnots() << std::endl;
		TColStd_Array1OfInteger multi = aBSpline->Multiplicities();
		{
			std::cout << "  - multiplicity messages: ";
			Standard_Integer lowerIndex1 = multi.Lower();
			Standard_Integer upperIndex1 = multi.Upper();
			for (Standard_Integer i = lowerIndex1; i <= upperIndex1; ++i) {
				// 使用 Value(i) 方法获取索引为 i 的点
				Standard_Real val = multi.Value(i);
				std::cout << "multiplicity:" << val << ",";
			}
			std::cout << std::endl;
		}
		{
			std::cout << "  - multiplicity knot messages: ";
			TColStd_Array1OfReal knots = aBSpline->Knots();
			Standard_Integer lowerIndex1 = knots.Lower();
			Standard_Integer upperIndex1 = knots.Upper();
			for (Standard_Integer i = lowerIndex1; i <= upperIndex1; ++i) {
				// 使用 Value(i) 方法获取索引为 i 的点
				Standard_Real val = knots.Value(i);
				std::cout << "multiply knot:" << val << ",";
			}
			std::cout << std::endl;
		}
		TColStd_Array1OfReal knots = aBSpline->KnotSequence();
		Standard_Integer lowerIndex1 = knots.Lower();
		Standard_Integer upperIndex1 = knots.Upper();
		std::cout << "  - knot sequence messages: ";
		for (Standard_Integer i = lowerIndex1; i <= upperIndex1; ++i) {
			// 使用 Value(i) 方法获取索引为 i 的点
			Standard_Real val = knots.Value(i);
			std::cout << "knot:" << val << ",";
		}
		std::cout << std::endl;
		do
		{
			std::cout << "  - uv sequence messages: " ;
			Standard_Real firstU = aBSpline->FirstParameter();
			Standard_Real lastU = aBSpline->LastParameter();
			std::cout << "firstU :" << firstU << "lastU " << lastU ;

			gp_Pnt pntU0, pntU1;
			//pntU0 = ComputePointOnCurve(0, poles, aBSpline->Degree(), knots);
			//pntU1 = ComputePointOnCurve(1, poles, aBSpline->Degree(), knots);
			aBSpline->D0(firstU, pntU0);
			aBSpline->D0(lastU, pntU1);
			std::cout << " pntU0 (" << pntU0.X() << " " << pntU0.Y() << " " << pntU0.Z() << ") ";
			std::cout << " pntU1 (" << pntU1.X() << " " << pntU1.Y() << " " << pntU1.Z() << ") ";
			std::cout << std::endl;
		} while (0);

		std::cout << "  - Is Periodic: " << (aBSpline->IsPeriodic() ? "True" : "False") << std::endl;
		std::cout << "  - Is Rational: " << (aBSpline->IsRational() ? "True" : "False") << std::endl;
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom_BezierCurve))) {
		Handle(Geom_BezierCurve) aBezier = Handle(Geom_BezierCurve)::DownCast(theCurve);
		std::cout << "  - Bezier Degree: " << aBezier->Degree() << std::endl;
		std::cout << "  - Number of Poles: " << aBezier->NbPoles() << std::endl;
		std::cout << "  - Is Rational: " << (aBezier->IsRational() ? "True" : "False") << std::endl;
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom_TrimmedCurve))) {
		Handle(Geom_TrimmedCurve) aTrimmed = Handle(Geom_TrimmedCurve)::DownCast(theCurve);
		Handle(Geom_Curve) basisCurve = aTrimmed->BasisCurve();
		std::cout << "  - Basis Curve Type: " << basisCurve->DynamicType()->Name() << std::endl;
		std::cout << "  - Trimmed Parameters: [" << aTrimmed->FirstParameter() << ", " << aTrimmed->LastParameter() << "]" << std::endl;
		// 可选：递归调用以输出基准曲线的信息
		// std::cout << "\n  --- Basis Curve Info ---" << std::endl;
		// PrintGeomCurveInfo(basisCurve, theParamSample);
	}
	else if (theCurve->IsKind(STANDARD_TYPE(Geom_OffsetCurve))) {
		Handle(Geom_OffsetCurve) anOffset = Handle(Geom_OffsetCurve)::DownCast(theCurve);
		Handle(Geom_Curve) basisCurve = anOffset->BasisCurve();
		std::cout << "  - Offset Distance: " << anOffset->Offset() << std::endl;
		std::cout << "  - Basis Curve Type: " << basisCurve->DynamicType()->Name() << std::endl;
		// 可选：递归调用以输出基准曲线的信息
		// std::cout << "\n  --- Basis Curve Info ---" << std::endl;
		// PrintGeomCurveInfo(basisCurve, theParamSample);
	}
	else {
		std::cout << "  - Specific details for this curve type are not implemented in this function." << std::endl;
	}
	std::cout << std::endl;
}

// fill primitive arrays
void GetEdgeCurve(TopoDS_Shape myshape)
{
	// 遍历所有的边
	TopExp_Explorer anExplorer;

	for (anExplorer.Init(myshape, TopAbs_FACE); anExplorer.More(); anExplorer.Next())
	{
		const TopoDS_Face& fc = TopoDS::Face(anExplorer.Current());
		GetFaceSurfaceInfo(fc);
		TopExp_Explorer edgeExplorer;
		for (edgeExplorer.Init(fc, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
		{
			const TopoDS_Edge& anEdge = TopoDS::Edge(edgeExplorer.Current());
			GetCurveInfoFromEdge(anEdge);
			TopLoc_Location loc = anEdge.Location();
			Standard_Real fd, ld;
			Handle(Geom_Curve) curve = BRep_Tool::Curve(anEdge, fd, ld);
			std::cout << "-- edge curve interval:" << fd<<"," << ld << std::endl;
			PrintGeomCurveInfo(curve);
			Standard_Real First, Last;
			const Handle(Geom2d_Curve) aCurve = BRep_Tool::CurveOnSurface(anEdge, fc, First, Last);
			std::cout << "-- edge on surfce curve interval:" << First << "," << Last << std::endl;
			PrintGeneralCurveInfo(aCurve);
			//Handle(Geom2d_TrimmedCurve) aTrimmedCurve = new Geom2d_TrimmedCurve(aCurve, First, First);
		}
		// 处理边
	}


}


void TraverseShape(const TopoDS_Shape& aShape)
{
	// 创建一个拓扑遍历器
	TopExp_Explorer anExplorer;

	// 遍历所有的面
	for (anExplorer.Init(aShape, TopAbs_FACE); anExplorer.More(); anExplorer.Next())
	{
		const TopoDS_Face& aFace = TopoDS::Face(anExplorer.Current());
		// 处理面
	}

	// 遍历所有的边
	for (anExplorer.Init(aShape, TopAbs_EDGE); anExplorer.More(); anExplorer.Next())
	{
		const TopoDS_Edge& anEdge = TopoDS::Edge(anExplorer.Current());
		// 处理边
	}

	// 遍历所有的顶点
	for (anExplorer.Init(aShape, TopAbs_VERTEX); anExplorer.More(); anExplorer.Next())
	{
		const TopoDS_Vertex& aVertex = TopoDS::Vertex(anExplorer.Current());
		// 处理顶点
	}
}

bool CompareShapes(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2) {
	// 比较形状的类型
	if (shape1.ShapeType() != shape2.ShapeType()) {
		return false;
	}

	// 根据形状类型，使用不同的遍历策略
	switch (shape1.ShapeType()) {
	case TopAbs_SOLID:
	case TopAbs_SHELL: {
		// 对于实体和壳，比较它们的面
		TopExp_Explorer explorer1(shape1, TopAbs_FACE);
		TopExp_Explorer explorer2(shape2, TopAbs_FACE);

		while (explorer1.More() && explorer2.More()) {
			if (!CompareShapes(TopoDS::Face(explorer1.Current()), TopoDS::Face(explorer2.Current()))) {
				return false;
			}
			explorer1.Next();
			explorer2.Next();
		}
		if (explorer1.More() || explorer2.More()) {
			return false; // 一个形状有更多的面
		}
		break;
	}
	case TopAbs_FACE: {
		// 对于面，比较它们的边
		TopExp_Explorer explorer1(shape1, TopAbs_EDGE);
		TopExp_Explorer explorer2(shape2, TopAbs_EDGE);

		while (explorer1.More() && explorer2.More()) {
			if (!CompareShapes(TopoDS::Edge(explorer1.Current()), TopoDS::Edge(explorer2.Current()))) {
				return false;
			}
			explorer1.Next();
			explorer2.Next();
		}
		if (explorer1.More() || explorer2.More()) {
			return false; // 一个形状有更多的边
		}
		break;
	}
	case TopAbs_EDGE: {
		// 对于边，比较它们的顶点
		TopExp_Explorer explorer1(shape1, TopAbs_VERTEX);
		TopExp_Explorer explorer2(shape2, TopAbs_VERTEX);

		while (explorer1.More() && explorer2.More()) {
			if (BRepTools::Compare(TopoDS::Vertex(explorer1.Current()), TopoDS::Vertex(explorer2.Current())) != 0) {
				return false;
			}
			explorer1.Next();
			explorer2.Next();
		}
		if (explorer1.More() || explorer2.More()) {
			return false; // 一个形状有更多的顶点
		}
		break;
	}
	case TopAbs_VERTEX: {
		// 对于顶点，比较它们的坐标
		gp_Pnt pnt1 = BRep_Tool::Pnt(TopoDS::Vertex(shape1));
		gp_Pnt pnt2 = BRep_Tool::Pnt(TopoDS::Vertex(shape2));
		return pnt1.IsEqual(pnt2, Precision::Confusion());
	}
	default:
		// 其他类型的形状，暂时不比较
		break;
	}

	return true;
}

