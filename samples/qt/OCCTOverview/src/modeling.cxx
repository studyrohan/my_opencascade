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
#include "BRepBuilderAPI_MakeWire.hxx"
#include "gp_Torus.hxx"
#include "BRepBuilderAPI_MakeShell.hxx"
#include "BRepBuilderAPI_MakeSolid.hxx"

#include <Geom_BSplineSurface.hxx> // 样条曲面头文件
#include <TColgp_Array2OfPnt.hxx>   // 控制点数组头文件
#include <iostream>
#include "BRepPrimAPI_MakeCylinder.hxx"

#include <Geom_BezierCurve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepProj_Projection.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax3.hxx>
#include <gp_Cylinder.hxx>

#include <AIS_Axis.hxx>
#include <AIS_ColoredShape.hxx>
#include <AIS_Plane.hxx>
#include <AIS_Point.hxx>
#include <AIS_TextLabel.hxx>
#include "GeomFill_Line.hxx"
#include "GeomFill_SectionGenerator.hxx"
#include "Geom_BezierCurve.hxx"
#include "GeomFill_Coons.hxx"
#include "GeomFill_BezierCurves.hxx"
#include "Geom_BezierSurface.hxx"
#include "GeomConvert.hxx"
#include "GeomFill_BSplineCurves.hxx"
#include "GeomAPI.hxx"
#include "Geom2dAPI_Interpolate.hxx"
#include "Geom2d_TrimmedCurve.hxx"
#include "GeomFill_AppSurf.hxx"
#include "TColGeom_Array1OfCurve.hxx"
#include "BRepBuilderAPI_MakeFace.hxx"
#include "GeomFill_Pipe.hxx"
#include "Extrema_ExtPS.hxx"
#include "GeomAdaptor_Surface.hxx"
#include "BRepBuilderAPI_Sewing.hxx"
#include <gp_Pnt.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Extrema_ExtPC.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <gp_Pnt2d.hxx>
#include <iostream>
#include <GeomAPI_IntCS.hxx>
#include "BasicFucntion.h"




static TopoDS_Shape projection();
static TopoDS_Shape fill();
static TopoDS_Shape cylinder();
static TopoDS_Shape point_projection_plane();
static TopoDS_Shape point_projection_bsurf();
static Handle(Geom_BSplineSurface) CreateBSplineSurface();
static TopoDS_Shape rollBall();
static TopoDS_Shape sewFace();
static void ComputePointToNURBSDistance();
static void CurveCurveIntersectionExample();
static void CurveSurfaceIntersectionExample();
static GeomAbs_Shape CheckContinuity(
	const Handle(Geom_Curve)& curve1,
	const Handle(Geom_Curve)& curve2,
	double tolerance);
static void testContinity();

TopoDS_Shape runMyCode()
{
	int index = 4;
	switch (index)
	{
	case 0:
	{// Step 1: 创建几条用于蒙皮的 B 样条曲线（截面线）

	// 曲线 1：一条水平直线
		GeomFill_SectionGenerator section;
		TColgp_Array1OfPnt curve1(1, 4);
		curve1.SetValue(1, gp_Pnt(0, 0, 0));
		curve1.SetValue(2, gp_Pnt(1, 0, 0));
		curve1.SetValue(3, gp_Pnt(2, 0, 0));
		curve1.SetValue(4, gp_Pnt(3, 0, 0));
		Handle(Geom_BSplineCurve) curveA = GeomAPI_PointsToBSpline(curve1).Curve();

		// 曲线 2：Y 方向偏移
		TColgp_Array1OfPnt curve2(1, 4);
		curve2.SetValue(1, gp_Pnt(0, 1, 0));
		curve2.SetValue(2, gp_Pnt(1, 1, 0.5));
		curve2.SetValue(3, gp_Pnt(2, 1, 0.5));
		curve2.SetValue(4, gp_Pnt(3, 1, 0));
		Handle(Geom_BSplineCurve) curveB = GeomAPI_PointsToBSpline(curve2).Curve();

		// 曲线 3：Z 方向提升
		TColgp_Array1OfPnt curve3(1, 4);
		curve3.SetValue(1, gp_Pnt(0, 2, 1));
		curve3.SetValue(2, gp_Pnt(1, 2, 1.5));
		curve3.SetValue(3, gp_Pnt(2, 2, 1.5));
		curve3.SetValue(4, gp_Pnt(3, 2, 1));
		Handle(Geom_BSplineCurve) curveC = GeomAPI_PointsToBSpline(curve3).Curve();

		// Step 2: 将曲线放入数组中

		section.AddCurve(curveA);
		section.AddCurve(curveB);
		section.AddCurve(curveC);

		// Step 3: 设置逼近参数
		Standard_Integer degmin = 3;            // 最小次数
		Standard_Integer degmax = 8;            // 最大次数
		Standard_Real tolerance = Precision::Confusion(); // 容差

		// Step 4: 蒙皮构造曲面
		section.Perform(Precision::PConfusion());

		GeomFill_AppSurf appSurf(3, degmin, degmax, tolerance, tolerance);
		appSurf.SetContinuity(GeomAbs_C1);
		//appSurf.Surface()
		//Handle(Geom_Surface) surface = appSurf.Surface();
		//if (appSurf.IsDone()) {
		//    Handle(Geom_Surface) surface = appSurf.Surface();
		//    std::cout << "✅ 曲面构造成功！" << std::endl;

		//    // 可选：转换为 BSpline 曲面以便访问更多信息
		//    Handle(Geom_BSplineSurface) bsplSurface = Handle(Geom_BSplineSurface)::DownCast(surface);
		//    if (!bsplSurface.IsNull()) {
		//        std::cout << "控制点行数: " << bsplSurface->NbVPoles() << std::endl;
		//        std::cout << "控制点列数: " << bsplSurface->NbUPoles() << std::endl;
		//    }
		//}
		//else {
		//    std::cerr << "❌ 曲面构造失败！" << std::endl;
		//};
		//
		break;
	}
	case 1:
	{
		// Step 1: 创建四条边界曲线（Bezier 曲线为例）

		// 前边（u=0）
		TColgp_Array1OfPnt frontPoles(1, 4);
		frontPoles.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
		frontPoles.SetValue(2, gp_Pnt(1.0, 0.0, 0.5));
		frontPoles.SetValue(3, gp_Pnt(2.0, 0.0, 0.5));
		frontPoles.SetValue(4, gp_Pnt(3.0, 0.0, 0.0));
		Handle(Geom_BezierCurve) front = new Geom_BezierCurve(frontPoles);

		// 后边（u=1）
		TColgp_Array1OfPnt backPoles(1, 4);
		backPoles.SetValue(1, gp_Pnt(0.0, 3.0, 0.0));
		backPoles.SetValue(2, gp_Pnt(1.0, 3.0, 1.0));
		backPoles.SetValue(3, gp_Pnt(2.0, 3.0, 1.0));
		backPoles.SetValue(4, gp_Pnt(3.0, 3.0, 0.0));
		Handle(Geom_BezierCurve) back = new Geom_BezierCurve(backPoles);

		// 左边（v=0）
		TColgp_Array1OfPnt leftPoles(1, 4);
		leftPoles.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
		leftPoles.SetValue(2, gp_Pnt(0.0, 1.0, 0.5));
		leftPoles.SetValue(3, gp_Pnt(0.0, 2.0, 0.5));
		leftPoles.SetValue(4, gp_Pnt(0.0, 3.0, 0.0));
		Handle(Geom_BezierCurve) left = new Geom_BezierCurve(leftPoles);

		// 右边（v=1）
		TColgp_Array1OfPnt rightPoles(1, 4);
		rightPoles.SetValue(1, gp_Pnt(3.0, 0.0, 0.0));
		rightPoles.SetValue(2, gp_Pnt(3.0, 1.0, 0.5));
		rightPoles.SetValue(3, gp_Pnt(3.0, 2.0, 0.5));
		rightPoles.SetValue(4, gp_Pnt(3.0, 3.0, 0.0));
		Handle(Geom_BezierCurve) right = new Geom_BezierCurve(rightPoles);

		// Step 2: 将四条曲线放入数组
		TColGeom_Array1OfCurve boundaryCurves(1, 4);
		boundaryCurves.SetValue(1, front);   // u = 0
		boundaryCurves.SetValue(2, back);    // u = 1
		boundaryCurves.SetValue(3, left);    // v = 0
		boundaryCurves.SetValue(4, right);   // v = 1

		// Step 3: 构造 Coons 曲面
		GeomFill_Coons coons;
		coons.Init(frontPoles, rightPoles, backPoles, leftPoles);
		Standard_Boolean isRat = coons.isRational();
		Standard_Integer NbUPoles = coons.NbUPoles();
		Standard_Integer NbVPoles = coons.NbVPoles();
		TColgp_Array2OfPnt Poles(1, NbUPoles, 1, NbVPoles);

		coons.Poles(Poles);
		Standard_Integer Lower = Poles.LowerCol();
		Standard_Integer Upper = Poles.UpperCol();
		Standard_Integer Length = Poles.ColLength();
		Standard_Integer i = Poles.LowerRow();
		gp_Pnt pnt = Poles.ChangeValue(1, 1);
		gp_XYZ xyz = pnt.Coord();
		xyz.Add({ 0,1,2 });
		/*          TColStd_Array2OfReal Weights;
				  coons.Weights(Weights);*/

				  /*TColStd_Array1OfReal    UKnots(1, NbUKnot);
				  TColStd_Array1OfInteger UMults(1, NbUKnot);
				  Standard_Integer NbUKnot = front->NbKnots();
				  CC1->Knots(UKnots);
				  CC1->Multiplicities(UMults);

				  Standard_Integer NbVKnot = CC2->NbKnots();
				  TColStd_Array1OfReal    VKnots(1, NbVKnot);
				  TColStd_Array1OfInteger VMults(1, NbVKnot);
				  CC2->Knots(VKnots);
				  CC2->Multiplicities(VMults);

				  TColStd_Array2OfReal Weights(1, NbUPoles, 1, NbVPoles);
				  Handle(Geom_BSplineSurface) mySurface = new Geom_BSplineSurface(Poles, Weights,
					  UKnots, VKnots,
					  UMults, VMults,
					  front->Degree(), leftPoles->Degree());*/
					  //coons.

					  //Handle(Geom_Surface) surface = coons.Surface();
					  //const TColgp_Array2OfPnt& Poles,
					  //    const TColStd_Array1OfReal& UKnots, 
					  //    const TColStd_Array1OfReal& VKnots, 
					  //    const TColStd_Array1OfInteger& UMults, 
					  //    const TColStd_Array1OfInteger& VMults,
					  //    const Standard_Integer UDegree, 
					  //    const Standard_Integer VDegree,
					  //    const Standard_Boolean UPeriodic = Standard_False, 
					  //    const Standard_Boolean VPeriodic = Standard_False
					  //Handle(Geom_BSplineSurface) aBSplineSurf = new Geom_BSplineSurface()

					  //// Step 4: 转换为 BSpline 曲面以便获取详细信息
					  //Handle(Geom_BSplineSurface) bsplSurf = Handle(Geom_BSplineSurface)::DownCast(surface);
					  //if (bsplSurf.IsNull()) {
					  //    std::cerr << "❌ 曲面不是 BSpline 曲面类型！" << std::endl;
					  //    return ;
					  //}

					  //// Step 5: 获取控制点、节点向量、次数等信息
					  //Standard_Integer nu = bsplSurf->NbUPoles();
					  //Standard_Integer nv = bsplSurf->NbVPoles();
					  //Standard_Integer degU = bsplSurf->UDegree();
					  //Standard_Integer degV = bsplSurf->VDegree();

					  //std::cout << "曲面次数 U: " << degU << ", V: " << degV << std::endl;
					  //std::cout << "控制点数量 U: " << nu << ", V: " << nv << std::endl;

					  //// 输出控制点
					  //std::cout << "\ 控制点列表:\n";
					  //for (Standard_Integer i = 1; i <= nu; ++i) {
					  //    for (Standard_Integer j = 1; j <= nv; ++j) {
					  //        gp_Pnt pt = bsplSurf->Pole(i, j);
					  //        std::cout << "P(" << i << "," << j << ") = ("
					  //            << pt.X() << ", " << pt.Y() << ", " << pt.Z() << ")\n";
					  //    }
					  //}

					  //// 输出节点向量 U 和 V
					  //std::cout << "\n 节点向量 U:\n";
					  //const TColStd_Array1OfReal& knotsU = bsplSurf->UKnots();
					  //for (Standard_Integer k = knotsU.Lower(); k <= knotsU.Upper(); ++k) {
					  //    std::cout << knotsU(k) << " ";
					  //}
					  //std::cout << std::endl;

					  //std::cout << "\n 节点向量 V:\n";
					  //const TColStd_Array1OfReal& knotsV = bsplSurf->VKnots();
					  //for (Standard_Integer k = knotsV.Lower(); k <= knotsV.Upper(); ++k) {
					  //    std::cout << knotsV(k) << " ";
					  //}
					  //std::cout << std::endl;
		break;
	}
	case 3:
	{
		// 前边（u=0）
		TColgp_Array1OfPnt frontPoles(1, 4);
		frontPoles.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
		frontPoles.SetValue(2, gp_Pnt(1.0, 0.0, 0.5));
		frontPoles.SetValue(3, gp_Pnt(2.0, 0.0, 0.5));
		frontPoles.SetValue(4, gp_Pnt(3.0, 0.0, 0.0));
		Handle(Geom_BezierCurve) front = new Geom_BezierCurve(frontPoles);

		// 后边（u=1）
		TColgp_Array1OfPnt backPoles(1, 4);
		backPoles.SetValue(1, gp_Pnt(0.0, 3.0, 0.0));
		backPoles.SetValue(2, gp_Pnt(1.0, 3.0, 1.0));
		backPoles.SetValue(3, gp_Pnt(2.0, 3.0, 1.0));
		backPoles.SetValue(4, gp_Pnt(3.0, 3.0, 0.0));
		Handle(Geom_BezierCurve) back = new Geom_BezierCurve(backPoles);

		// 左边（v=0）
		TColgp_Array1OfPnt leftPoles(1, 4);
		leftPoles.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
		leftPoles.SetValue(2, gp_Pnt(0.0, 1.0, 0.5));
		leftPoles.SetValue(3, gp_Pnt(0.0, 2.0, 0.5));
		leftPoles.SetValue(4, gp_Pnt(0.0, 3.0, 0.0));
		Handle(Geom_BezierCurve) left = new Geom_BezierCurve(leftPoles);

		// 右边（v=1）
		TColgp_Array1OfPnt rightPoles(1, 4);
		rightPoles.SetValue(1, gp_Pnt(3.0, 0.0, 0.0));
		rightPoles.SetValue(2, gp_Pnt(3.0, 1.0, 0.5));
		rightPoles.SetValue(3, gp_Pnt(3.0, 2.0, 0.5));
		rightPoles.SetValue(4, gp_Pnt(3.0, 3.0, 0.0));
		Handle(Geom_BezierCurve) right = new Geom_BezierCurve(rightPoles);
		GeomFill_BezierCurves geomFill;
		geomFill.Init(front, right, back, left, GeomFill_FillingStyle::GeomFill_CoonsStyle);
		Handle(Geom_BezierSurface) srf = geomFill.Surface();
		Standard_Integer NbUPoles = srf->NbUPoles();
		Standard_Integer NbVPoles = srf->NbVPoles();

		gp_Pnt pnt;
		srf->D0(0, 0, pnt);
		break;
	}
	case 4:
	{
		return fill();
		break;
	}
	case 5:
	{
		return cylinder();
		break;
	}
	case 6:
	{
		return point_projection_bsurf();
	}
	case 7:
	{
		return point_projection_plane();
	}
	case 8:
	{
		return  rollBall();
	}
	case 9:
	{
		ComputePointToNURBSDistance();
		CurveCurveIntersectionExample();
		testContinity();
		return sewFace();
	}
	case 10:
	{
		testBasizerFunction();
		break;
	}
	default:
		break;
	}

	return TopoDS_Shape();
}

#include <GeomAdaptor_Curve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <ElCLib.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <iostream>
#include <BRep_Tool.hxx>
#include <BRepLProp_SLProps.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Precision.hxx>
/*​获取共享边与相邻曲面​：首先需要找到两个曲面(Face1, Face2) 之间的共享边(SharedEdge)。
​在共享边上采样点​：沿着共享边获取一系列参数点，用于后续的逐点检查。
​计算曲面的微分几何属性​：对于每个采样点，在两张曲面上分别计算其法向量、主曲率（最大曲率 k1 和最小曲率 k2）以及主曲率方向。
​比较属性判断连续性​：
​G0​：检查两点位置是否重合（距离小于容差）。
​G1​：在 G0 基础上，检查法向量方向是否平行（夹角余弦接近 1）。
​G2​：在 G1 基础上，检查主曲率 k1, k2 是否相等（差值小于容差），并且主曲率方向是否一致。*/

bool CheckG2Continuity(const TopoDS_Face& face1, const TopoDS_Face& face2, const TopoDS_Edge& sharedEdge, double tolerance) {
	// 获取共享边的几何曲线和参数范围
	Standard_Real first, last;
	Handle(Geom_Curve) curve = BRep_Tool::Curve(sharedEdge, first, last);
	GeomAdaptor_Curve gac(curve, first, last);

	// 在边上创建采样点
	GCPnts_QuasiUniformDeflection pointGenerator;
	pointGenerator.Initialize(gac, tolerance); // 使用容差控制采样密度
	if (!pointGenerator.IsDone()) { return false; }

	int numPoints = pointGenerator.NbPoints();
	if (numPoints < 2) { return false; }

	// 准备曲面几何属性计算工具，需要计算到二阶导数 (Curvature)
	BRepLProp_SLProps props1(face1, 2, tolerance); // 计算到二阶导数
	BRepLProp_SLProps props2(face2, 2, tolerance);

	for (int i = 1; i <= numPoints; ++i) {
		gp_Pnt pointOnEdge = pointGenerator.Value(i);
		double param = pointGenerator.Parameter(i);

		// 获取点在两个曲面上的参数 (UV 坐标)
		gp_Pnt2d uv1, uv2;
		// 注意：这里需要根据你的具体模型获取点在曲面上的参数坐标(UV)
		// 这通常可以通过 BRep_Tool::UVPoints 或类似方法实现
		// 假设我们已经获得了 uv1 和 uv2

		// 设置属性计算工具的参数点
		props1.SetParameters(uv1.X(), uv1.Y());
		props2.SetParameters(uv2.X(), uv2.Y());

		// 1. 检查 G0: 位置连续
		if (!props1.Value().IsEqual(props2.Value(), tolerance)) {
			return false; // 位置不连续，未达到G0
		}

		// 2. 检查 G1: 法向连续 (相切连续)
		if (!props1.Normal().IsParallel(props2.Normal(), tolerance)) {
			return false; // 法向不平行，未达到G1
		}

		// 3. 检查 G2: 曲率连续
		// 检查高斯曲率是否相等 (绝对值差小于容差)
		if (Abs(props1.GaussianCurvature() - props2.GaussianCurvature()) > tolerance) {
			return false;
		}
		// 检查平均曲率是否相等 (绝对值差小于容差)
		if (Abs(props1.MeanCurvature() - props2.MeanCurvature()) > tolerance) {
			return false;
		}
		// 更严格的检查可以比较主曲率的方向
		// if (!props1.MaxCurvatureDirection().IsParallel(props2.MaxCurvatureDirection(), tolerance)) {
		//     return false;
		// }
		// if (!props1.MinCurvatureDirection().IsParallel(props2.MinCurvatureDirection(), tolerance)) {
		//     return false;
		// }
	}

	// 所有采样点均通过 G0, G1, G2 检查
	return true;
}

/*在微分几何中，G²连续性（曲率连续）要求在两个曲面沿着公共边界相遇时，满足以下条件：

​位置连续 (G⁰)​​：边界上的点完全重合。
​切线连续 (G¹)​​：边界上任意一点处，两个曲面具有相同的切平面。
​曲率连续 (G²)​​：边界上任意一点处，两个曲面在跨越边界时曲率的变化是连续的。这意味着曲面的曲率半径保持一致，没有突变，从而形成极其平滑的过渡
。*/
/*这个函数可以被看作是一个信息检索工具。它读取的是在建模操作（如布尔运算、倒角、曲面缝合）
中预先定义并存储在拓扑数据结构中的连续性标签或属性
，而不是在调用时实时进行计算判断*/

GeomAbs_Shape GetFaceContinuity(const TopoDS_Face& face1, const TopoDS_Face& face2, const TopoDS_Edge& sharedEdge) {
	// 使用 BRep_Tool 获取拓扑边上存储的连续性信息
	GeomAbs_Shape continuity = BRep_Tool::Continuity(sharedEdge, face1, face2);
	return continuity;
}

// 函数：检查两条曲线在连接点的连续性
GeomAbs_Shape CheckContinuity(
	const Handle(Geom_Curve)& curve1,
	const Handle(Geom_Curve)& curve2,
	double tolerance)
{
	// 获取第一条曲线的终点参数和第二条曲线的起点参数
	double u1_end = curve1->LastParameter();
	double u2_begin = curve2->FirstParameter();

	// 计算连接点处的点位置
	gp_Pnt P1, P2;
	curve1->D0(u1_end, P1);
	curve2->D0(u2_begin, P2);

	// 1. 检查G0连续性：点位置是否重合
	if (P1.Distance(P2) > tolerance) {
		std::cout << "不满足G0连续，点距离为: " << P1.Distance(P2) << std::endl;
		return GeomAbs_C0; // 实际上连G0都不满足
	}
	std::cout << "满足G0连续" << std::endl;

	// 计算一阶导数（切线向量）
	gp_Vec tangent1, tangent2;
	curve1->D1(u1_end, P1, tangent1);
	curve2->D1(u2_begin, P2, tangent2);

	// 2. 检查G1连续性：切线方向是否相同（忽略模长）
	if (!tangent1.IsParallel(tangent2, tolerance)) {
		std::cout << "满足G0但不满足G1连续" << std::endl;
		return GeomAbs_G1;
	}
	std::cout << "满足G1连续" << std::endl;

	// 计算二阶导数（用于曲率计算）
	gp_Vec d21, d22;
	curve1->D2(u1_end, P1, tangent1, d21);
	curve2->D2(u2_begin, P2, tangent2, d22);

	// 计算曲率矢量
	// 曲率k = |C' × C''| / |C'|^3，但这里我们主要比较方向
	gp_Vec normal1 = tangent1.Normalized();
	gp_Vec normal2 = tangent2.Normalized();

	// 3. 检查G2连续性：曲率是否连续
	// 简化检查：比较归一化的法向量方向[6](@ref)
	if (!normal1.IsParallel(normal2, tolerance)) {
		std::cout << "满足G1但不满足G2连续" << std::endl;
		return GeomAbs_G2;
	}
	std::cout << "满足G2连续" << std::endl;

	return GeomAbs_C2; // 至少满足G2
}



void testContinity()
{
	// 定义第一条曲线的控制点
	TColgp_Array1OfPnt points1(1, 3);
	points1.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
	points1.SetValue(2, gp_Pnt(2.0, 3.0, 0.0));
	points1.SetValue(3, gp_Pnt(5.0, 1.0, 0.0)); // 这条曲线的终点，也是第二条曲线的起点

	// 使用GeomAPI_PointsToBSpline拟合第一条B样条曲线
	GeomAPI_PointsToBSpline approximator1;
	approximator1.Init(points1);
	Handle(Geom_BSplineCurve) curve1 = approximator1.Curve();

	// 确保第二条曲线的起点与第一条曲线的终点重合，并设计控制点以保持切线方向
	TColgp_Array1OfPnt points2(1, 3);
	points2.SetValue(1, gp_Pnt(5.0, 1.0, 0.0)); // 与curve1的终点重合，保证G0连续

	// 计算第一条曲线终点的切线方向
	gp_Pnt P_end;
	gp_Vec tangent_end;
	curve1->D1(curve1->LastParameter(), P_end, tangent_end);

	// 第二条曲线的第二个控制点沿着切线方向放置，以尝试实现G1连续
	// 这里通过将切向量缩放后加到终点坐标上来估算第二个控制点
	gp_Vec scaledTangent = 2.0 * tangent_end.Normalized(); // 缩放因子可根据需要调整
	points2.SetValue(2, P_end.Translated(scaledTangent));

	points2.SetValue(3, gp_Pnt(8.0, 4.0, 0.0)); // 第二个曲线的终点

	GeomAPI_PointsToBSpline approximator2;
	approximator2.Init(points2);
	Handle(Geom_BSplineCurve) curve2 = approximator2.Curve();

	double tolerance = 1e-6;
	GeomAbs_Shape continuity = CheckContinuity(curve1, curve2, tolerance);

	switch (continuity) {
	case GeomAbs_C0: std::cout << "最终连续性结果: G0" << std::endl; break;
	case GeomAbs_G1: std::cout << "最终连续性结果: G1" << std::endl; break;
	case GeomAbs_G2: std::cout << "最终连续性结果: G2" << std::endl; break;
	default: std::cout << "连续性高于G2" << std::endl;
	}

	// 可选：将曲线转换为TopoDS_Edge并生成Wire进行可视化或进一步操作[1,5](@ref)
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(curve1);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(curve2);
	TopoDS_Wire wire = BRepBuilderAPI_MakeWire(edge1, edge2);

	std::cout << "两条曲线已构造并连接成Wire。" << std::endl;
}


Handle(Geom_BSplineSurface) CreateBSplineSurface()
{
	// 定义网格尺寸
	const Standard_Integer numRows = 4; // U 方向点数
	const Standard_Integer numCols = 4; // V 方向点数

	// 创建点阵容器 (注意: OCCT 数组索引通常从 1 开始)
	TColgp_Array2OfPnt points(1, numRows, 1, numCols);

	// 生成示例数据（一个简单的鞍形曲面）
	for (Standard_Integer i = 1; i <= numRows; ++i)
	{
		double u = (i - 1) / static_cast<double>(numRows - 1);
		for (Standard_Integer j = 1; j <= numCols; ++j)
		{
			double v = (j - 1) / static_cast<double>(numCols - 1);
			double x = u * 100.0;
			double y = v * 100.0;
			double z = 10.0 * (u - 0.5) * (v - 0.5); // 简单的鞍形函数

			points.SetValue(i, j, gp_Pnt(x, y, z));
		}
	}

	// 创建 B 样条曲面插值器
	GeomAPI_PointsToBSplineSurface surfaceBuilder;
	surfaceBuilder.Init(points);

	// 检查是否成功
	if (!surfaceBuilder.IsDone())
	{
		// 处理错误
		throw Standard_Failure("Failed to create B-spline surface");
	}

	// 获取生成的 B 样条曲面
	Handle(Geom_BSplineSurface) surface = surfaceBuilder.Surface();

	return surface;
	//// 如果需要将几何曲面转换为拓扑面 (TopoDS_Face)
	//BRepBuilderAPI_MakeFace faceMaker(surface, Precision::Confusion());
	//if (!faceMaker.IsDone())
	//{
	//	throw Standard_Failure("Failed to create face from surface");
	//}

	//return faceMaker.Face();
}

TopoDS_Shape fill()
{
	// Outline
	double height = 8.5;
	TColgp_Array1OfPnt outer_e_bzr_geom_v(1, 4);
	gp_Pnt JiZhunXian2_v0 = gp_Pnt(-17.6, 0.0, 0.0);
	gp_Pnt JiZhunXian2_v1 = gp_Pnt(0, 32.8, 0.0);
	{
		outer_e_bzr_geom_v(1) = JiZhunXian2_v0;
		outer_e_bzr_geom_v(4) = JiZhunXian2_v1;

		Standard_Real ratio1 = 5.4 / 13.2;
		outer_e_bzr_geom_v(2) = gp_Pnt(outer_e_bzr_geom_v(1).X(), ratio1 * outer_e_bzr_geom_v(4).Y(), 0);
		Standard_Real ratio2 = 6.0 / 6.8;
		outer_e_bzr_geom_v(3) = gp_Pnt(ratio2 * outer_e_bzr_geom_v(1).X(), outer_e_bzr_geom_v(4).Y(), 0);
	}

	Handle(Geom_BezierCurve) outer_e_bzr_geom = new Geom_BezierCurve(outer_e_bzr_geom_v);
	Handle(Geom_BSplineCurve) outer_e_bsp_geom = GeomConvert::CurveToBSplineCurve(outer_e_bzr_geom);
	TopoDS_Edge outer_e = BRepBuilderAPI_MakeEdge(outer_e_bsp_geom);

	Handle(Geom_BSplineCurve) curve1;
	{
		Handle(TColgp_HArray1OfPnt2d) harray = new TColgp_HArray1OfPnt2d(1, 2); // sizing harray
		harray->SetValue(1, gp_Pnt2d(-JiZhunXian2_v1.Y(), 0));
		harray->SetValue(2, gp_Pnt2d(0, height + height / 2));

		Geom2dAPI_Interpolate anInterpolation(harray, Standard_False, 1e-6);

		gp_Vec2d vtangent1(0, 1);
		gp_Vec2d vtangent2(1, 0);
		anInterpolation.Load(vtangent1, vtangent2);
		anInterpolation.Perform();

		Handle(Geom2d_BSplineCurve) c = anInterpolation.Curve();

		gp_Pln pln(gp_Ax3(gp_Pnt(), gp_Dir(1, 0, 0), gp_Dir(0, -1, 0)));

		Handle(Geom_BSplineCurve) c3d = Handle(Geom_BSplineCurve)::DownCast(GeomAPI::To3d(c, pln));
		curve1 = c3d;
	}

	Handle(Geom_BSplineCurve) curve2;
	{
		Handle(TColgp_HArray1OfPnt2d) harray = new TColgp_HArray1OfPnt2d(1, 3); // sizing harray
		harray->SetValue(1, gp_Pnt2d(-JiZhunXian2_v0.X(), 0));
		harray->SetValue(2, gp_Pnt2d(-JiZhunXian2_v0.X() - 2.6, height));
		harray->SetValue(3, gp_Pnt2d(0, height + height / 2));

		Geom2dAPI_Interpolate anInterpolation(harray, Standard_False, 1e-6);
		anInterpolation.Perform();

		Handle(Geom2d_BSplineCurve) c = anInterpolation.Curve();
		gp_Pln pln(gp_Ax3(gp_Pnt(), gp_Dir(0, -1, 0), gp_Dir(-1, 0, 0)));
		Handle(Geom_BSplineCurve) c3d = Handle(Geom_BSplineCurve)::DownCast(GeomAPI::To3d(c, pln));
		curve2 = c3d;
	}
	GeomFill_BSplineCurves fill2;
	fill2.Init(outer_e_bsp_geom, curve1, curve2, GeomFill_CoonsStyle);

	const Handle(Geom_BSplineSurface)& surf_geom = fill2.Surface();

	TopoDS_Shape filled_face = BRepBuilderAPI_MakeFace(surf_geom, 0);

	GetEdgeCurve(filled_face);
	return filled_face;
}

TopoDS_Shape cylinder()
{
	gp_Torus aTorus(gp::XOY(), 20.0, 7.5);
	Handle(Geom_Surface) aTorusSurf = new Geom_ToroidalSurface(aTorus);
	TopoDS_Shell aTorusShell = BRepBuilderAPI_MakeShell(aTorusSurf, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI);
	// Make a solid on the torus shell.
	TopoDS_Solid aTorusSolid = BRepBuilderAPI_MakeSolid(aTorusShell);

	Standard_Real aRadius1 = 5.0;
	Standard_Real aLength1 = 15.0;
	TopoDS_Shape aCyl1 = BRepPrimAPI_MakeCylinder(aRadius1, aLength1);
	GetEdgeCurve(aCyl1);
	return aCyl1;
}

TopoDS_Shape point_projection_bsurf()
{
	// 1. 定义待投影的点 (gp_Pnt)
	gp_Pnt aPoint(1.0, 2.0, 3.0);

	Handle(Geom_BSplineSurface)srf = CreateBSplineSurface();
	// 3. 定义曲面参数范围 (UMin, UMax, VMin, VMax) 和计算公差 (Tolerance)
	Standard_Real UMin = 0.0;
	Standard_Real UMax = 10.0;
	Standard_Real VMin = 0.0;
	Standard_Real VMax = 10.0;
	Standard_Real Tolerance = 1.0e-6;
	GeomAdaptor_Surface AS(srf);

	// 4. 创建极值计算器并执行计算
	Extrema_ExtPS anExtrema(aPoint, AS, UMin, UMax, VMin, VMax, Tolerance, Tolerance);

	// 6. 获取极值点数量
	Standard_Integer nbExt = anExtrema.NbExt();
	if (nbExt == 0)
	{
		std::cout << "No extremum point found." << std::endl;
		return TopoDS_Shape();
	}

	// 7. 寻找最小距离的极值点 (投影点)
	Standard_Integer minIndex = 1;
	Standard_Real minSquareDist = anExtrema.SquareDistance(1);
	for (Standard_Integer i = 2; i <= nbExt; ++i)
	{
		if (anExtrema.SquareDistance(i) < minSquareDist)
		{
			minSquareDist = anExtrema.SquareDistance(i);
			minIndex = i;
		}
	}

	// 8. 获取最近投影点的信息
	Extrema_POnSurf pOnSurf = anExtrema.Point(minIndex);
	gp_Pnt nearestPoint = pOnSurf.Value(); // 投影点的三维坐标
	Standard_Real U, V;
	pOnSurf.Parameter(U, V); // 投影点在曲面上的参数坐标 (U, V)
	Standard_Real distance = sqrt(minSquareDist); // 实际距离

	std::cout << "Projection Point Coordinates: ("
		<< nearestPoint.X() << ", "
		<< nearestPoint.Y() << ", "
		<< nearestPoint.Z() << ")" << std::endl;
	std::cout << "Parameters on Surface (U, V): ("
		<< U << ", " << V << ")" << std::endl;
	std::cout << "Distance: " << distance << std::endl;

	return BRepBuilderAPI_MakeFace(srf,Tolerance);

}
TopoDS_Shape point_projection_plane()
{
	// 1. 定义待投影的点 (gp_Pnt)
	gp_Pnt aPoint(1.0, 2.0, 3.0);

	// 2. 获取目标曲面 (Handle(Geom_Surface))
	// 这里以创建一个简单的平面为例，你需要替换为你的实际曲面
	Handle(Geom_Plane) aSurface = new Geom_Plane(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)); // XOY平面

	// 3. 定义曲面参数范围 (UMin, UMax, VMin, VMax) 和计算公差 (Tolerance)
	Standard_Real UMin = 0.0;
	Standard_Real UMax = 10.0;
	Standard_Real VMin = 0.0;
	Standard_Real VMax = 10.0;
	Standard_Real Tolerance = 1.0e-6;
	GeomAdaptor_Surface AS(aSurface);

	// 4. 创建极值计算器并执行计算
	Extrema_ExtPS anExtrema(aPoint, AS, UMin, UMax, VMin, VMax, Tolerance, Tolerance);

	// 5. 检查计算是否成功
	if (!anExtrema.IsDone())
	{
		std::cerr << "Extrema calculation failed!" << std::endl;
		return TopoDS_Shape();
	}

	// 6. 获取极值点数量
	Standard_Integer nbExt = anExtrema.NbExt();
	if (nbExt == 0)
	{
		std::cout << "No extremum point found." << std::endl;
		return TopoDS_Shape();
	}

	// 7. 寻找最小距离的极值点 (投影点)
	Standard_Integer minIndex = 1;
	Standard_Real minSquareDist = anExtrema.SquareDistance(1);
	for (Standard_Integer i = 2; i <= nbExt; ++i)
	{
		if (anExtrema.SquareDistance(i) < minSquareDist)
		{
			minSquareDist = anExtrema.SquareDistance(i);
			minIndex = i;
		}
	}

	// 8. 获取最近投影点的信息
	Extrema_POnSurf pOnSurf = anExtrema.Point(minIndex);
	gp_Pnt nearestPoint = pOnSurf.Value(); // 投影点的三维坐标
	Standard_Real U, V;
	pOnSurf.Parameter(U, V); // 投影点在曲面上的参数坐标 (U, V)
	Standard_Real distance = sqrt(minSquareDist); // 实际距离

	std::cout << "Projection Point Coordinates: ("
		<< nearestPoint.X() << ", "
		<< nearestPoint.Y() << ", "
		<< nearestPoint.Z() << ")" << std::endl;
	std::cout << "Parameters on Surface (U, V): ("
		<< U << ", " << V << ")" << std::endl;
	std::cout << "Distance: " << distance << std::endl;

	return BRepBuilderAPI_MakeFace(aSurface,0,10,0,10,Tolerance);
}

TopoDS_Shape projection()
{
	// 1. 定义一条贝塞尔曲线

	TColgp_Array1OfPnt backPoles(1, 8);
	backPoles.SetValue(1, gp_Pnt(0.0, -100.0, 0.0));
	backPoles.SetValue(1, gp_Pnt(10.0, 10.0, 0.0));
	backPoles.SetValue(1, gp_Pnt(30.0, -100.0, 0.0));
	backPoles.SetValue(1, gp_Pnt(100.0, 0.0, 0.0));
	backPoles.SetValue(1, gp_Pnt(150.0, 50.0, 0.0));
	backPoles.SetValue(1, gp_Pnt(200.0, 0.0, 0.0));
	backPoles.SetValue(1, gp_Pnt(400.0, 200.0, 0.0));
	backPoles.SetValue(1, gp_Pnt(450.0, 0.0, 0.0));
	Handle(Geom_BezierCurve) bezierCurve = new Geom_BezierCurve(backPoles);
	TopoDS_Edge curveEdge = BRepBuilderAPI_MakeEdge(bezierCurve);

	// 2. 定义一个圆柱面
	gp_Ax3 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)); // 在YOZ平面上的圆柱，沿Y轴
	Handle(Geom_CylindricalSurface) cylinder = new Geom_CylindricalSurface(axis, 200.0);
	// 创建圆柱面的有限面部（指定参数范围）
	TopoDS_Face cylinderFace = BRepBuilderAPI_MakeFace(cylinder->Cylinder(), 0, 2 * M_PI, -200, 500);

	// 3. 定义投影方向（Z轴方向）
	gp_Dir projectionDir(0.0, 0.0, 1.0);

	// 4. 创建投影对象并获取投影结果
	BRepProj_Projection projector(curveEdge, cylinderFace, projectionDir);

	// 5. 检查投影结果并获取投影曲线
	if (projector.More()) // 检查是否有投影结果
	{
		TopoDS_Shape projectedShape = projector.Current();

		// 5. 获取投影得到的Edge（这是曲面上的3D曲线）
		TopoDS_Edge projectedEdge = TopoDS::Edge(projector.Current());

		// 6. 关键步骤：从投影后的Edge和曲面Face获取PCurve
		Standard_Real firstParam, lastParam;
		Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(projectedEdge, cylinderFace, firstParam, lastParam);

		if (pcurve.IsNull()) {
			std::cerr << "Failed to get PCurve!" << std::endl;
	
		}

		// 7. 示例：输出PCurve的起点和终点参数
		gp_Pnt2d startPoint2d = pcurve->Value(firstParam);
		gp_Pnt2d endPoint2d = pcurve->Value(lastParam);

		std::cout << "PCurve参数范围: [" << firstParam << ", " << lastParam << "]" << std::endl;
		std::cout << "起点参数坐标 (U,V): (" << startPoint2d.X() << ", " << startPoint2d.Y() << ")" << std::endl;
		std::cout << "终点参数坐标 (U,V): (" << endPoint2d.X() << ", " << endPoint2d.Y() << ")" << std::endl;
		// projectedShape 即为投影得到的曲线（Edge）
		// 这里可以进一步处理投影结果，例如保存或显示
		GetEdgeCurve(projectedShape);
		return projectedShape;
	}

	return TopoDS_Shape();
}

TopoDS_Shape rollBall()
{
	try {
		// 1. 创建路径曲线 (一条简单的B样条曲线)
		TColgp_Array1OfPnt pathPoints(1, 3);
		pathPoints.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
		pathPoints.SetValue(2, gp_Pnt(50.0, 100.0, 0.0));
		pathPoints.SetValue(3, gp_Pnt(100.0, 0.0, 0.0));

		Handle(Geom_BSplineCurve) pathCurve;
		GeomAPI_PointsToBSpline pathInterpolator;
		pathInterpolator.Init(pathPoints);
		if (pathInterpolator.IsDone()) {
			pathCurve = pathInterpolator.Curve();
		}
		else {
			std::cerr << "Failed to create path curve!" << std::endl;
			return TopoDS_Shape();
		}

		// 2. 创建第一条轨道曲线 (例如，一个椭圆弧)
		gp_Ax2 ellipseAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
		Handle(Geom_Ellipse) ellipse1 = new Geom_Ellipse(ellipseAxis, 30.0, 15.0);
		Handle(Geom_TrimmedCurve) railCurve1 = new Geom_TrimmedCurve(ellipse1, 0, M_PI); // 上半椭圆

		// 3. 创建第二条轨道曲线 (另一个椭圆弧，可与第一条不同)
		Handle(Geom_Ellipse) ellipse2 = new Geom_Ellipse(ellipseAxis, 40.0, 20.0);
		Handle(Geom_TrimmedCurve) railCurve2 = new Geom_TrimmedCurve(ellipse2, 0, M_PI); // 上半椭圆

		// 4. 定义滚球半径
		Standard_Real radius = 10.0;

		// 5. 创建滚球曲面
		GeomFill_Pipe pipe;
		pipe.Init(pathCurve, 10);
		//pipe.Init(pathCurve, railCurve1, railCurve2); // 使用路径、两条轨道和半径初始化

		Standard_Real tolerance = 1.0e-6;
		pipe.Perform(tolerance); // 执行计算

		if (!pipe.IsDone()) {
			std::cerr << "Failed to create rolling ball surface!" << std::endl;
			return TopoDS_Shape();
		}

		// 6. 获取生成的滚球曲面
		Handle(Geom_Surface) rollingBallSurface = pipe.Surface();

		// 7. (可选) 将几何曲面转换为拓扑面以便后续操作或可视化
		// 注意：这里需要指定曲面的参数范围，此处使用一个近似值，实际应根据曲面类型调整
		Standard_Real uMin, uMax, vMin, vMax;
		rollingBallSurface->Bounds(uMin, uMax, vMin, vMax);
		TopoDS_Face face = BRepBuilderAPI_MakeFace(rollingBallSurface, uMin, uMax, vMin, vMax, tolerance);

		std::cout << "Rolling ball surface created and saved successfully!" << std::endl;
		return face;
	}
	catch (...)
	{

	}
	return TopoDS_Shape();
}

// 示例：通过添加控制点来延伸B样条曲线（简单模拟）
Handle(Geom_BSplineCurve) ExtendBSplineCurve(
	const Handle(Geom_BSplineCurve)& originalCurve,
	Standard_Real startExtension,
	Standard_Real endExtension,
	Standard_Integer numExtraPoints = 1)
{
	// 获取原始曲线的控制点
	Standard_Integer numPoles = originalCurve->NbPoles();
	TColgp_Array1OfPnt poles(1, numPoles);
	originalCurve->Poles(poles);

	// 计算延伸方向（这里使用曲线端点的切线方向进行线性延伸）
	gp_Pnt firstPole = poles.Value(1);
	gp_Pnt lastPole = poles.Value(numPoles);
	gp_Vec startDir(poles.Value(2), firstPole); // 大致起始方向
	gp_Vec endDir(poles.Value(numPoles - 1), lastPole); // 大致末端方向

	// 扩展控制点数组：在起始和末端添加新点以模拟延伸
	Standard_Integer newLength = numPoles + numExtraPoints * 2;
	TColgp_Array1OfPnt newPoles(1, newLength);

	// 添加新的起始延伸点
	for (Standard_Integer i = 1; i <= numExtraPoints; ++i)
	{
		gp_Pnt newStartPoint = firstPole.Translated(startDir * (startExtension * i / numExtraPoints));
		newPoles.SetValue(i, newStartPoint);
	}

	// 复制原始控制点
	for (Standard_Integer i = 1; i <= numPoles; ++i)
	{
		newPoles.SetValue(i + numExtraPoints, poles.Value(i));
	}

	// 添加新的末端延伸点
	for (Standard_Integer i = 1; i <= numExtraPoints; ++i)
	{
		gp_Pnt newEndPoint = lastPole.Translated(endDir * (endExtension * i / numExtraPoints));
		newPoles.SetValue(numPoles + numExtraPoints + i, newEndPoint);
	}

	// 使用扩展后的控制点重新拟合B样条曲线
	GeomAPI_PointsToBSpline curveFitter;
	curveFitter.Init(newPoles);
	if (curveFitter.IsDone())
	{
		return curveFitter.Curve();
	}
	else
	{
		std::cerr << "曲线拟合失败！" << std::endl;
		return nullptr;
	}
}

TopoDS_Shape extendCurve()
{
	// 1. 创建一条简单的示例B样条曲线
	TColgp_Array1OfPnt points(1, 4);
	points.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
	points.SetValue(2, gp_Pnt(5.0, 5.0, 0.0));
	points.SetValue(3, gp_Pnt(10.0, 3.0, 0.0));
	points.SetValue(4, gp_Pnt(15.0, 0.0, 0.0));

	GeomAPI_PointsToBSpline curveBuilder;
	curveBuilder.Init(points);
	if (!curveBuilder.IsDone())
	{
		std::cerr << "创建曲线失败！" << std::endl;
		return TopoDS_Shape();
	}
	Handle(Geom_BSplineCurve) originalCurve = curveBuilder.Curve();

	// 2. 延伸曲线：起始端延伸5个单位，末端延伸10个单位
	Standard_Real startExtend = 5.0;
	Standard_Real endExtend = 10.0;
	Handle(Geom_BSplineCurve) extendedCurve = ExtendBSplineCurve(originalCurve, startExtend, endExtend, 2);

	if (extendedCurve.IsNull())
	{
		std::cerr << "曲线延伸失败！" << std::endl;
		return TopoDS_Shape();
	}

	// 3. 此处可以使用延伸后的曲线进行后续操作
	TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(extendedCurve);

	std::cout << "曲线延伸完成！" << std::endl;
	return edge;

}

TopoDS_Shape sewFace()
{
	// 创建两个独立的平面
	Handle(Geom_Plane) plane1 = new Geom_Plane(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1));
	Handle(Geom_Plane) plane2 = new Geom_Plane(gp_Pnt(0, 10, 0), gp_Vec(0, 0, 1));

	TopoDS_Face face1 = BRepBuilderAPI_MakeFace(plane1, 0, 10, 0, 10, 1e-6);
	TopoDS_Face face2 = BRepBuilderAPI_MakeFace(plane2, 0, 10, 0, 10, 1e-6);

	// 创建缝合工具
	BRepBuilderAPI_Sewing sewer(1.0e-5);

	// 添加面
	sewer.Add(face1);
	sewer.Add(face2);

	// 执行缝合
	sewer.Perform();

	if (!sewer.SewedShape().IsNull()) {
		TopoDS_Shape result = sewer.SewedShape();
		std::cout << "缝合成功!" << std::endl;
		std::cout << "自由边数量: " << sewer.NbFreeEdges() << std::endl;
		// 可以进一步操作result
		return result;
	}
	else {
		std::cerr << "缝合失败." << std::endl;
	}
	return TopoDS_Shape();
}



void ComputePointToNURBSDistance()
{
	// 1. 创建一组点来构造示例性的NURBS曲线
	TColgp_Array1OfPnt points(1, 5);
	points.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
	points.SetValue(2, gp_Pnt(1.0, 2.0, 1.0));
	points.SetValue(3, gp_Pnt(2.0, 1.0, 2.0));
	points.SetValue(4, gp_Pnt(4.0, 3.0, 1.0));
	points.SetValue(5, gp_Pnt(5.0, 0.0, 0.0));

	// 使用点列拟合3D B样条曲线
	GeomAPI_PointsToBSpline fitter;
	fitter.Init(points);
	Handle(Geom_BSplineCurve) nurbsCurve = fitter.Curve();

	// 2. 定义要计算距离的点
	gp_Pnt testPoint(3.0, 1.5, 1.0);

	// 3. 使用GeomAdaptor_Curve适配曲线用于极值计算
	GeomAdaptor_Curve curveAdaptor(nurbsCurve);

	// 4. 创建Extrema_ExtPC对象并执行计算
	Extrema_ExtPC extremaCalculator(testPoint, curveAdaptor);
	extremaCalculator.Perform(testPoint);

	// 5. 检查结果并输出最小距离
	if (extremaCalculator.IsDone() && extremaCalculator.NbExt() > 0)
	{
		// 找到所有极值点中的最小距离
		Standard_Real minSquareDist = extremaCalculator.SquareDistance(1); // 获取第一个极值的平方距离
		int minIndex = 1;
		for (int i = 2; i <= extremaCalculator.NbExt(); i++)
		{
			Standard_Real sqDist = extremaCalculator.SquareDistance(i);
			if (sqDist < minSquareDist)
			{
				minSquareDist = sqDist;
				minIndex = i;
			}
		}

		// 计算实际距离（开平方根）
		Standard_Real minDistance = sqrt(minSquareDist);

		// 获取最小距离对应的曲线参数及投影点坐标
		Extrema_POnCurv projPoint = extremaCalculator.Point(minIndex);
		Standard_Real curveParam = projPoint.Parameter();
		gp_Pnt closestPoint = projPoint.Value();

		std::cout << "最小距离: " << minDistance << std::endl;
		std::cout << "曲线参数U: " << curveParam << std::endl;
		std::cout << "曲线上最近点坐标: ("
			<< closestPoint.X() << ", "
			<< closestPoint.Y() << ", "
			<< closestPoint.Z() << ")" << std::endl;
	}
	else
	{
		std::cout << "极值计算未完成或未找到解。" << std::endl;
	}
}

void CurveCurveIntersectionExample()
{
	// 1. 创建第一条NURBS曲线 (示例: 通过点列拟合)
	TColgp_Array1OfPnt2d points1(1, 3);
	points1.SetValue(1, gp_Pnt2d(0.0, 0.0));
	points1.SetValue(2, gp_Pnt2d(5.0, 5.0));
	points1.SetValue(3, gp_Pnt2d(10.0, 0.0));

	//  knots, multiplicities 等参数可根据需要设置，这里简单处理
	Handle(Geom2d_BSplineCurve) curve1 = Geom2dAPI_PointsToBSpline(points1).Curve();

	// 2. 创建第二条NURBS曲线
	TColgp_Array1OfPnt2d points2(1, 3);
	points2.SetValue(1, gp_Pnt2d(0.0, 5.0));
	points2.SetValue(2, gp_Pnt2d(5.0, 0.0));
	points2.SetValue(3, gp_Pnt2d(10.0, 5.0));

	Handle(Geom2d_BSplineCurve) curve2 = Geom2dAPI_PointsToBSpline(points2).Curve();

	// 3. 定义求交容差 (可选，默认1.0e-6)
	Standard_Real tolerance = 1.0e-6;

	// 4. 创建求交对象并计算
	Geom2dAPI_InterCurveCurve intersector(curve1, curve2, tolerance);

	// 5. 检查求交是否成功完成
	if (intersector.NbPoints()>0)
	{
		// 6. 获取交点数量
		Standard_Integer numPoints = intersector.NbPoints();
		std::cout << "Number of intersection points found: " << numPoints << std::endl;

		if (numPoints > 0)
		{
			// 7. 遍历所有交点
			for (Standard_Integer i = 1; i <= numPoints; ++i)
			{
				// 获取交点坐标
				gp_Pnt2d point = intersector.Point(i);
				std::cout << "Intersection Point " << i << ": ("
					<< point.X() << ", " << point.Y() << ")" << std::endl;

				// 如果需要获取交点在曲线上的参数值，可以使用更底层的接口
				// 例如通过 intersector.Intersector() 获取 Geom2dInt_GInter 对象
				// 然后从 IntRes2d_IntersectionPoint 中获取参数
				// 注意: Geom2dAPI_InterCurveCurve 的 Point 方法直接返回的是坐标
				// 获取参数值通常需要访问底层求交结果 (参见后面的补充说明)
			}
		}
		else
		{
			std::cout << "No intersection points found." << std::endl;
		}

		// 8. 检查是否有重合段 (例如两曲线部分重叠)
		Standard_Integer numSegments = intersector.NbSegments();
		if (numSegments > 0)
		{
			std::cout << "Number of overlapping segments found: " << numSegments << std::endl;
			// 可以进一步使用 Segment() 方法获取重合段的曲线表示
		}
	}
	else
	{
		std::cout << "Intersection calculation failed or was not performed." << std::endl;
	}
}

void CurveSurfaceIntersectionExample()
{
	// 1. 创建一条B样条曲线 (示例)
	TColgp_Array1OfPnt curvePoints(1, 3);
	curvePoints.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
	curvePoints.SetValue(2, gp_Pnt(5.0, 5.0, 5.0));
	curvePoints.SetValue(3, gp_Pnt(10.0, 10.0, 10.0));

	// 使用点列拟合曲线 (这里简化了参数设置)
	Handle(Geom_BSplineCurve) curve = GeomAPI_PointsToBSpline(curvePoints).Curve();

	// 2. 创建一个B样条曲面 (示例)
	TColgp_Array2OfPnt surfPoints(1, 3, 1, 3);
	surfPoints.SetValue(1, 1, gp_Pnt(0.0, 0.0, 0.0));
	surfPoints.SetValue(1, 2, gp_Pnt(0.0, 5.0, 0.0));
	surfPoints.SetValue(1, 3, gp_Pnt(0.0, 10.0, 0.0));
	surfPoints.SetValue(2, 1, gp_Pnt(5.0, 0.0, 5.0));
	surfPoints.SetValue(2, 2, gp_Pnt(5.0, 5.0, 5.0));
	surfPoints.SetValue(2, 3, gp_Pnt(5.0, 10.0, 5.0));
	surfPoints.SetValue(3, 1, gp_Pnt(10.0, 0.0, 10.0));
	surfPoints.SetValue(3, 2, gp_Pnt(10.0, 5.0, 10.0));
	surfPoints.SetValue(3, 3, gp_Pnt(10.0, 10.0, 10.0));

	// 使用点阵拟合曲面 (这里简化了参数设置)
	Handle(Geom_BSplineSurface) surface = GeomAPI_PointsToBSplineSurface(surfPoints).Surface();

	// 3. 创建求交对象并执行计算
	GeomAPI_IntCS intersector(curve, surface);

	// 4. 检查求交是否成功完成
	if (intersector.IsDone())
	{
		// 5. 获取交点数量
		Standard_Integer numPoints = intersector.NbPoints();
		std::cout << "Number of intersection points found: " << numPoints << std::endl;

		// 6. 遍历并输出所有交点坐标
		for (Standard_Integer i = 1; i <= numPoints; ++i)
		{
			gp_Pnt intersectionPoint = intersector.Point(i);
			std::cout << "Point " << i << ": ("
				<< intersectionPoint.X() << ", "
				<< intersectionPoint.Y() << ", "
				<< intersectionPoint.Z() << ")" << std::endl;
		}

		// 7. 检查是否有相交线段 (例如曲线部分位于曲面上)
		Standard_Integer numSegments = intersector.NbSegments();
		std::cout << "Number of intersection segments found: " << numSegments << std::endl;
		if (numSegments > 0)
		{
			for (Standard_Integer i = 1; i <= numSegments; ++i)
			{
				Handle(Geom_Curve) segmentCurve = intersector.Segment(i);
				// 可以对得到的线段进行进一步处理或显示
				std::cout << "Retrieved segment " << i << std::endl;
			}
		}
	}
	else
	{
		std::cout << "Intersection calculation failed or was not performed." << std::endl;
	}
}

