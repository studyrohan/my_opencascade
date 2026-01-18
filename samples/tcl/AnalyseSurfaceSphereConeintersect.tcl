# 加载所有插件（确保命令可用）
pload ALL

# 创建几何曲面
# 语法：plane <名称> <点X> <点Y> <点Z> <法向X> <法向Y> <法向Z>
plane pln 0 0 1 0 0 1

# 语法：sphere <名称> <中心X> <中心Y> <中心Z> <半径>
sphere sph 0 0 0 5

# 创建拓扑面（用于布尔运算）
# 语法：mkface <面名称> <曲面名称> [U参数范围] [V参数范围]
mkface f_pln pln -10 10 -10 10
mkface f_sph sph 0 2*pi 0 pi

# 执行布尔求交操作（获取交线）
# 语法：bop <形状1> <形状2>
bop f_pln f_sph

# 提取并显示交线结果
# 语法：bopsection <结果名称>
bopsection r

# 可视化
vinit
vdisplay f_pln f_sph r
vsetdispmode r 1
vfit