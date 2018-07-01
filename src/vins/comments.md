##VINS-Mono代码分析总结
Tags： SLAM
****
[TOC]
****
###参考文献
[1] [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen (techincal report)](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/support_files/paper/tro_technical_report.pdf)    
[2] [Solà J. Quaternion kinematics for the error-state KF[M]// Surface and Interface Analysis. 2015.](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf)</span>   
[3] [Solà J. Yang Z, Shen S. Monocular Visual–Inertial State Estimation With Online Initialization and Camera–IMU Extrinsic Calibration[J]. IEEE Transactions on Automation Science & Engineering, 2017, 14(1):39-51.](http://ieeexplore.ieee.org/document/7463059/?reload=true&arnumber=7463059)</span>     
[4] [Engel J, Schöps T, Cremers D. LSD-SLAM: Large-scale direct monocular SLAM[C]//European Conference on Computer Vision. Springer, Cham, 2014:834-849.](http://xueshu.baidu.com/s?wd=paperuri%3A%28b5b76005594ab735af07c8a12035cc20%29&filter=sc_long_sign&tn=SE_xueshusource_2kduw22v&sc_vurl=http%3A%2F%2Flink.springer.com%2F10.1007%2F978-3-319-10605-2_54&ie=utf-8&sc_us=18106927782980728032)</span>    
[5] [Shen S, Michael N, Kumar V. Tightly-coupled monocular visual-inertial fusion for autonomous flight of rotorcraft MAVs[C]// IEEE International Conference on Robotics and Automation. IEEE, 2015:5303-5310.](http://xueshu.baidu.com/s?wd=paperuri%3A%28bd3b80a1f1df33cc04a61dfd0eb02a59%29&filter=sc_long_sign&tn=SE_xueshusource_2kduw22v&sc_vurl=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Ficp.jsp%3Farnumber%3D7139939&ie=utf-8&sc_us=6750941297362817987)
****
###前言      
&emsp;&emsp;Vins-Mono是视觉与IMU的融合中的经典之作，其定位精度可以媲美OKVIS，而且具有比OKVIS更加完善和鲁棒的初始化以及闭环检测过程。同时VINS-Mono也为该邻域树立了一个信标吧，视觉SLAM的研究和应用会更新偏向于 __单目+IMU__。因为在机器人的导航中，尤其是无人机的自主导航中，单目不具有RGBD相机(易受光照影响、获取的深度信息有限)以及双目相机(占用较大的空间。。。)的限制，能够适应室内、室外及不同光照的环境，具有较好的适应性。而且在增强和虚拟现实中，更多的移动设备仅具有单个相机，所以单目+IMU也更符合实际情况。      
&emsp;&emsp;那么为什么要进行视觉与IMU的融合呢，自己总结的主要有以下几点：    

- 视觉与IMU的融合可以借助IMU较高的采样频率，进而提高系统的输出频率。
- 视觉与IMU的融合可以提高视觉的鲁棒性，如视觉SLAM因为某些运动或场景出现的错误结果。
- 视觉与IMU的融合可以有效的消除IMU的积分漂移。
- 视觉与IMU的融合能够校正IMU的Bias。
- 单目与IMU的融合可以有效解决单目尺度不可观测的问题。

&emsp;&emsp;上面总结了视觉与IMU融合的几个优点，以及与单目融合可以解决单目相机尺度不可测的问题。但是单目相机的尺度不可测也是具有优点的，参考[[4]](#[4])。单目尺度不确定的优点主要有两方面：单目尺度的不确定性，可以对不同的规模大小的环境空间之间进行游走切换，祈祷无缝连接的作用，比如从室内桌面上的环境和大规模的户外场景；诸如深度或立体视觉相机，这些具备深度信息的传感器，它们所提供的可靠深度信息范围，是有限制的，故而不能像单目相机那样具有尺度灵活性的特点。
###1 预积分的推导 
####1.1 离散状态下预积分方程      
&emsp;&emsp;关于这部分的论文和代码中的推导，可以参考文献[[2]](#[2])中Appendx部分“A Runge-Kutta numerical integration methods”中的欧拉法和中值法。
$$
w_{k}^{{}'}=\frac{w_{k+1}+w_{k}}{2}-b_{w}
\tag{1.1}
$$
$$
q _{i+1}=q _{i}\otimes \begin{bmatrix}
1
\\
0.5w_{k}^{{}'}
\end{bmatrix}
\tag{1.2}
$$
$$
a_{k}^{{}'}=\frac{q_{k}(a_{k}+n_{a0}-b_{a_{k}})+q_{k+1}(a_{k+1}++n_{a1}-b_{a_{k}})}{2}
\tag{1.3}
$$
$$
\alpha _{i+1}=\delta\alpha _{i}+\beta _{i}t+0.5a_{k}^{{}'}\delta t^{2}
\tag{1.4}
$$
$$
\beta _{i+1}=\delta\beta _{i}+a_{k}^{{}'}\delta t
\tag{1.5}
$$

####1.2 离散状态下误差状态方程       
&emsp;&emsp;论文中Ⅱ.B部分的误差状态方程是连续时间域内，在实际代码中需要的是离散时间下的方程式，而且在前面的预积分方程中使用了中值法积分方式。所以在实际代码中和论文是不一致的。在推导误差状态方程式的最重要的部分是对 $\delta \theta _{k+1}$ 部分的推导。    
&emsp;&emsp;由泰勒公式可得：
$$
\delta \theta _{k+1} = \delta \theta _{k}+\dot{\delta \theta _{k}}\delta t
\tag{1.6}
$$
依据参考文献[[2]](#[2])中 "5.3.3 The error-state kinematics"中公式(222c)及其推导过程有：
$$
\dot{\delta \theta _{k}}=-[w_{m}-w_{b}]_{\times }\delta \theta _{k}-\delta w_{b}-w_{n}
$$
对于中值法积分下的误差状态方程为：
$$
\dot{\delta \theta _{k}}=-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta \theta _{k}-\delta b_{g_{k}}+\frac{n_{w0}+n_{w1}}{2}
\tag{1.7}
$$
将式(1.7)带入式(1.6)可得：
$$
\delta \theta _{k+1} =(I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t) \delta \theta _{k} -\delta b_{g_{k}}\delta t+\frac{n_{w0}+n_{w1}}{2}\delta t
\tag{1.8}
$$
这部分也可以参考，文献[[2]](#[2])中“7.2 System kinematics in discrete time”小节。    
&emsp;&emsp;接下来先推导 $\delta \beta _{k+1}$ 部分，再推导 $\delta \alpha _{k+1}$ 部分。$\delta \beta _{k+1}$ 部分的推导也可以参考文献[[2]](#[2])中“5.3.3 The error-state kinematics”公式(222b)的推导。将式(1.5)展开得到：
$$
\delta\beta _{i+1}=\delta\beta _{i}+\frac{q_{k}(a_{k}+n_{a0}-b_{a_{k}})+q_{k+1}(a_{k+1}++n_{a1}-b_{a_{k}})}{2}\delta t
$$
即，
$$
\delta\beta _{i+1}=\delta\beta _{i}+\dot{\delta\beta_{i}}\delta t
\tag{1.9}
$$
文献[2]中，公式(222b)
$$
\dot{\delta v}=-R[a_{m}-a_{b}]_{\times}\delta \theta-R\delta a_{b}+\delta g-Ra_{n}
$$
对于中值法积分下的误差状态方程为：
$$
\begin{align}\nonumber
\dot{\delta\beta_{i}} =&-\frac{1}{2}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta \theta-\frac{1}{2}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta \theta _{k+1} -\delta b_{g_{k}}\delta t+\frac{n_{w0}+n_{w1}}{2}\delta t)\delta \theta \\\nonumber
 &-\frac{1}{2}q_{k}\delta b_{a_{k}}-\frac{1}{2}q_{k+1}\delta b_{a_{k}}-\frac{1}{2}q_{k}n_{a0}-\frac{1}{2}q_{k}n_{a1}
 \end{align}
\tag{1.10}
$$
将式(1.8)带入式(1.10)可得
$$
\begin{align}\nonumber
\dot{\delta\beta_{i}} =&-\frac{1}{2}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta \theta-\frac{1}{2}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}((I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t) \delta \theta _{k} -\delta b_{g_{k}}\delta t+\frac{n_{w0}+n_{w1}}{2}\delta t) \\\nonumber
 &-\frac{1}{2}q_{k}\delta b_{a_{k}}-\frac{1}{2}q_{k+1}\delta b_{a_{k}}-\frac{1}{2}q_{k}n_{a0}-\frac{1}{2}q_{k}n_{a1}
 \end{align}
\tag{1.11}
$$
同理，可以计算出 $\delta \alpha _{k+1}$ ，可以写为：
$$
\delta\alpha _{i+1}=\delta\alpha _{i}+\dot{\delta\alpha_{i}}\delta t
\tag{1.12}
$$
$$
\begin{align}\nonumber
\dot{\delta\alpha_{i}} =&-\frac{1}{4}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta \theta\delta t-\frac{1}{4}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}((I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t) \delta \theta _{k} -\delta b_{g_{k}}\delta t+\frac{n_{w0}+n_{w1}}{2}\delta t)\delta t \\\nonumber
 &-\frac{1}{4}q_{k}\delta b_{a_{k}}\delta t-\frac{1}{4}q_{k+1}\delta b_{a_{k}}\delta t-\frac{1}{4}q_{k}n_{a0}\delta t-\frac{1}{4}q_{k}n_{a1}\delta t
 \end{align}
\tag{1.13}
$$
最后是加速度计和陀螺仪bias的误差状态方程，
$$
\delta b_{a_{k+1}}=\delta b_{a_{k}}+n_{ba}\delta t
\tag{1.14}
$$
$$
\delta b_{w_{k+1}}=\delta b_{w_{k}}+n_{bg}\delta t
\tag{1.15}
$$
&emsp;&emsp;综合式(1.8)等误差状态方程，将其写为矩阵形式，
$$
\begin{align}\nonumber
\begin{bmatrix}
\delta \alpha_{k+1}\\
\delta \theta  _{k+1}\\
\delta \beta _{k+1} \\
\delta b _{a{}{k+1}} \\
\delta b _{g{}{k+1}}
\end{bmatrix}&=\begin{bmatrix}
I & f_{01} &\delta t  & -\frac{1}{4}(q_{k}+q_{k+1})\delta t^{2} & f_{04}\\
0 & I-[\frac{w_{k+1}+w_{k}}{2}-b_{wk}]_{\times }\delta t & 0 &  0&-\delta t \\
0 &  f_{21}&I  &  -\frac{1}{2}(q_{k}+q_{k+1})\delta t & f_{24}\\
0 &  0&  0&I  &0 \\
 0& 0 & 0 & 0 & I
\end{bmatrix}
\begin{bmatrix}
\delta \alpha_{k}\\
\delta \theta  _{k}\\
\delta \beta _{k} \\
\delta b _{a{}{k}} \\
\delta b _{g{}{k}}
\end{bmatrix} \\\nonumber
&+
\begin{bmatrix}
 \frac{1}{4}q_{k}\delta t^{2}&  v_{01}& \frac{1}{4}q_{k+1}\delta t^{2} & v_{03} & 0 & 0\\
 0& \frac{1}{2}\delta t & 0 & \frac{1}{2}\delta t &0  & 0\\
 \frac{1}{2}q_{k}\delta t&  v_{21}& \frac{1}{2}q_{k+1}\delta t & v_{23} & 0 & 0 \\
0 & 0 & 0 & 0 &\delta t  &0 \\
 0& 0 &0  & 0 &0  & \delta t
\end{bmatrix}
\begin{bmatrix}
n_{a0}\\
n_{w0}\\
n_{a1}\\
n_{w1}\\
n_{ba}\\
n_{bg}
\end{bmatrix}
\end{align}
\tag{1.16}
$$
其中，
$$
\begin{align}\nonumber
f_{01}&=-\frac{1}{4}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta t^{2}-\frac{1}{4}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}(I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t)\delta t^{2} \\\nonumber
f_{21}&=-\frac{1}{2}q_{k}[a_{k}-b_{a_{k}}]_{\times}\delta t-\frac{1}{2}q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}(I-[\frac{w_{k+1}+w_{k}}{2}-b_{g_{k}}]_{\times }\delta t)\delta t \\\nonumber
f_{04}&=\frac{1}{4}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})(-\delta t) \\\nonumber
f_{24}&=\frac{1}{2}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t)(-\delta t) \\\nonumber
v_{01}&=\frac{1}{4}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})\frac{1}{2}\delta t \\\nonumber
v_{03}&=\frac{1}{4}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})\frac{1}{2}\delta t \\\nonumber
v_{21}&=\frac{1}{2}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})\frac{1}{2}\delta t \\\nonumber
v_{23}&=\frac{1}{2}(-q_{k+1}[a_{k+1}-b_{a_{k}}]_{\times}\delta t^{2})\frac{1}{2}\delta t
\end{align}
$$
将式(1.16)简写为，
$$
\delta z_{k+1} = F\delta z_{k}+VQ
$$
&emsp;&emsp;最后得到系统的雅克比矩阵 $J_{k+1}$ 和协方差矩阵 $P_{k+1}$，初始状态下的雅克比矩阵和协方差矩阵为单位阵和零矩阵，即
$$
J_{k}=I \\ P_{k}=0
$$
$$
J_{k+1}=FJ_{k}
\tag{1.17}
$$
$$
P_{k+1}=FP_{k}F^{T}+VQV_{T}
\tag{1.18}
$$

###2 前端KLT跟踪

###3 系统初始化 
&emsp;&emsp;在提取的图像的Features和做完IMU的预积分之后，进入了系统的初始化环节，那么系统为什么要进行初始化呢，主要的目的有以下两个：      

- 系统使用单目相机，如果没有一个良好的尺度估计，就无法对两个传感器做进一步的融合。这个时候需要恢复出尺度；
- 要对IMU进行初始化，IMU会受到bias的影响，所以要得到IMU的bias。

所以我们要从初始化中恢复出尺度、重力、速度以及IMU的bias，因为视觉(SFM)在初始化的过程中有着较好的表现，所以在初始化的过程中主要以SFM为主，然后将IMU的预积分结果与其对齐，即可得到较好的初始化结果。      
&emsp;&emsp;系统的初始化主要包括三个环节：求取相机与IMU之间的相对旋转、相机初始化(局部滑窗内的SFM，包括没有尺度的BA)、IMU与视觉的对齐(IMU预积分中的 $\alpha$等和相机的translation)。     
####3.1 相机与IMU之间的相对旋转     
&emsp;&emsp;这个地方相当于求取相机与IMU的一部分外参。相机与IMU之间的旋转标定非常重要，偏差1-2°系统的精度就会变的极低。这部分的内容参考文献[[3]](#[3])中Ⅴ-A部分，这里做简单的总结。      
&emsp;&emsp;设相机利用对极关系得到的旋转矩阵为 $R^{c_{k}}_{c_{k+1}}$，IMU经过预积分得到的旋转矩阵为$R^{b_{k}}_{b_{k+1}}$，相机与IMU之间的相对旋转为 $R^{b}_{c}$，则对于任一帧满足，
$$
R^{b_{k}}_{b_{k+1}}R^{b}_{c}=R^{b}_{c}R^{c_{k}}_{c_{k+1}}
\tag{3.1}
$$
对式(3.1)可以做简单的证明，在其两边同乘 $^{c}x^{k+1}$ 得
$$
\begin{align}\nonumber
x^{b}_{k}R^{b_{k}}_{b_{k+1}}R^{b}_{c}&=x^{b}_{k}R^{b}_{c}R^{c_{k}}_{c_{k+1}} \\\nonumber
x^{b}_{k+1}R^{b}_{c}&=x^{c}_{k}R^{c_{k}}_{c_{k+1}}
\\\nonumber
x^{c}_{k+1}&=x^{c}_{k+1}
\end{align}
$$
将旋转矩阵写为四元数，则式(3.1)可以写为
$$
q^{b_{k}}_{b{k+1}}\otimes q^{b}_{c}=q^{b}_{c}\otimes q^{c_{k}}_{c{k+1}}
$$
将其写为左乘和右乘的形式，综合为
$$
[Q_{1}(q^{b_{k}}_{b{k+1}})-Q_{2}(q^{c_{k}}_{c{k+1}})]q^{b}_{c}=Q^{k}_{k+1}q^{b}_{c}=0
\tag{3.2}
$$
其中 $Q_{1}(q^{b_{k}}_{b{k+1}})$，$Q_{2}(q^{c_{k}}_{c{k+1}})$ 分别表示四元数的左乘和右乘形式，
$$
\begin{align}\nonumber
Q_{1}(q)&=\begin{bmatrix}
q_{w}I_{3}+[q_{xyz }]_{\times} & q_{xyz}\\\nonumber
-q_{xyz} & q_{w}
\end{bmatrix} \\
Q_{2}(q)&=\begin{bmatrix}
q_{w}I_{3}-[q_{xyz }]_{\times} & q_{xyz}\\\nonumber
-q_{xyz} & q_{w}
\end{bmatrix}
\end{align}
\tag{3.3}
$$
那么对于 $n$对测量值，则有
$$
\begin{bmatrix}
w^{0}_{1}Q^{0}_{1}\\
w^{1}_{2}Q^{1}_{2}\\
\vdots \\
w^{N-1}_{N}Q^{N-1}_{N}
\end{bmatrix}q^{b}_{c}=Q_{N}q^{b}_{c}=0
\tag{3.4}
$$
其中 $w^{N-1}_{N}$ 为外点剔除权重，其与相对旋转求取得残差有关，$N$为计算相对旋转需要的测量对数，其由最终的终止条件决定。残差可以写为，
$$
r^{k}_{k+1}=acos((tr(\hat{R}^{b^{-1}}_{c}R^{b_{k}^{-1}}_{b_{k+1}}\hat{R}^{b}_{c}R^{c_{k}}_{c_{k+1}} )-1)/2)
\tag{3.5}
$$
残差还是很好理解的，在具体的代码中可以计算公式(3.1)两边两个旋转的得角度差。在得到残差之后就可以进一步得到公式(3.4)中的权重，
$$
w^{k}_{k+1}=\left\{\begin{matrix}
1,\qquad r^{k}_{k+1}<threshold\\
\frac{threshold}{r^{k}_{k+1}},\qquad otherwise
\end{matrix}\right.
\tag{3.6}
$$
一般会将阈值 $threshold$ 取做 $5°$。至此，就可以通过求解方程(3.4)得到相对旋转，式(3.4)的解为 $Q_{N}$ 的左奇异向量中最小奇异值对应的特征向量。     
&emsp;&emsp;但是，在这里还要注意 __求解的终止条件(校准完成的终止条件)__ 。在足够多的旋转运动中，我们可以很好的估计出相对旋转 $R^{b}_{c}$，这时 $Q_{N}$ 对应一个准确解，且其零空间的秩为1。但是在校准的过程中，某些轴向上可能存在退化运动(如匀速运动)，这时 $Q_{N}$ 的零空间的秩会大于1。判断条件就是 $Q_{N}$ 的第二小的奇异值是否大于某个阈值，若大于则其零空间的秩为1，反之秩大于1，相对旋转$R^{b}_{c}$ 的精度不够，校准不成功。      
####3.2 相机初始化      
####3.3 视觉与IMU对齐      
&emsp;&emsp;视觉与IMU的对齐主要解决三个问题：      
&emsp;&emsp;(1) 修正陀螺仪的bias；      
&emsp;&emsp;(2) 初始化速度、重力向量 $g$和尺度因子(Metric scale)；      
&emsp;&emsp;(3) 改进重力向量 $g$的量值；      
#####3.3.1 陀螺仪Bias修正     
&emsp;&emsp;__发现校正部分使用的都是一系列的约束条件，思路很重要啊__。陀螺仪Bias校正的时候也是使用了一个简单的约束条件：
$$
\underset{\delta b_{w}}{min}\sum_{k\in B}^{ }\left \| q^{c_{0}^{-1}}_{b_{k+1}}\otimes q^{c_{0}}_{b_{k}}\otimes\gamma _{b_{k+1}}^{b_{k}} \right \|^{2}
\tag{3.7}
$$
其中
$$
\gamma _{b_{k+1}}^{b_{k}}\approx \hat{\gamma}_{b_{k+1}}^{b_{k}}\otimes \begin{bmatrix}
1\\
\frac{1}{2}J^{\gamma }_{b_{w}}\delta b_{w}
\end{bmatrix}
\tag{3.8}
$$
公式(3.7)的最小值为单位四元数 $[1,0_{v}]^{T}$ ,所以将式(3.7)进一步写为，
$$
\begin{align}\nonumber
q^{c_{0}^{-1}}_{b_{k+1}}\otimes q^{c_{0}}_{b_{k}}\otimes\gamma _{b_{k+1}}^{b_{k}}&=\begin{bmatrix}
1\\
0
\end{bmatrix} \\\nonumber
\hat{\gamma}_{b_{k+1}}^{b_{k}}\otimes \begin{bmatrix}
1\\
\frac{1}{2}J^{\gamma }_{b_{w}}\delta b_{w}
\end{bmatrix}&=q^{c_{0}^{-1}}_{b_{k}}\otimes q^{c_{0}}_{b_{k+1}} \\
\end{align}
$$
$$
\begin{bmatrix}
1\\
\frac{1}{2}J^{\gamma }_{b_{w}}\delta b_{w}
\end{bmatrix}=\hat{\gamma}_{b_{k+1}}^{b_{k}^{-1}}\otimes q^{c_{0}^{-1}}_{b_{k}}\otimes q^{c_{0}}_{b_{k+1}}
\tag{3.9}
$$
只取式(3.9)式虚部，在进行最小二乘求解
$$
J^{\gamma^{T}}_{b_{w}}J^{\gamma }_{b_{w}}\delta b_{w}=J^{\gamma^{T}}_{b_{w}}(\hat{\gamma}_{b_{k+1}}^{b_{k}^{-1}}\otimes q^{c_{0}^{-1}}_{b_{k}}\otimes q^{c_{0}}_{b_{k+1}}).vec
\tag{3.10}
$$
求解式(3.10)的最小二乘解，即可得到 $\delta b_{w}$，注意这个地方得到的只是Bias的变化量，需要在滑窗内累加得到Bias的准确值。    
#####3.3.2 初始化速度、重力向量 $g$和尺度因子     
&emsp;&emsp;在这个步骤中，要估计系统的速度、重力向量以及尺度因子。所以系统的状态量可以写为，
$$
X_{I}=[v^{c_{0}}_{b_{0}},v^{c_{0}}_{b_{1}},\cdots ,g^{c_{0}},s]
\tag{3.11}
$$
上面的状态量都是在 $c_{0}$ 相机坐标系下。接着，有前面的由预积分部分，IMU的测量模型可知
$$
\begin{align}\nonumber
\alpha^{b_{k}}_{b_{k+1}}&=q^{b_{k}}_{c_{0}}(s(\bar{p}^{c_{0}}_{b_{k+1}}-\bar{p}^{c_{0}}_{b_{k}})+\frac{1}{2}g^{c_{0}}\triangle t_{k}^{2}-v^{c_{0}}_{b_{k}}\triangle t_{k}^{2}) \\\nonumber
\beta ^{b_{k}}_{b_{k+1}}&=q^{b_{k}}_{c_{0}}(v^{c_{0}}_{b_{k+1}}+g^{c_{0}}\triangle t_{k}-v^{c_{0}}_{b_{k}})
\end{align}
\tag{3.12}
$$
在3.1小节，我们已经得到了IMU相对于相机的旋转 $q_{b}^{c}$,假设IMU到相机的平移量$p_{b}^{c}$ 那么可以很容易地将相机坐标系下的位姿转换到IMU坐标系下，
$$
\begin{align}\nonumber
q_{b_{k}}^{c_{0}} &= q^{c_{0}}_{c_{k}}\otimes q_{b}^{c}  \\\nonumber
s\bar{p}^{c_{0}}_{b_{k}}&=s\bar{p}^{c_{0}}_{c_{k}}+q^{c_{0}}_{c_{k}}p_{b}^{c}
\end{align}
\tag{3.13}
$$
综合式(3.12)和式(3.13)可得，
$$
\begin{align}\nonumber
\hat{z}^{b_{k}}_{b_{k+1}}&=\begin{bmatrix}
\alpha^{b_{k}}_{b_{k+1}}-q^{c_{0}}_{c_{k+1}}p^{c}_{b}+q^{c_{0}}_{c_{k}}p^{c}_{b}&\\\nonumber
\beta ^{b_{k}}_{b_{k+1}}
\end{bmatrix}=H^{b_{k}}_{b_{k+1}}X_{I}+n^{b_{k}}_{b_{k+1}} \\\nonumber
&\approx \begin{bmatrix}
-q^{b_{k}}_{c_{0}}\triangle t_{k} &0&  1/2q^{b_{k}}_{c_{0}}\triangle t_{k}^{2} &q^{b_{k}}_{c_{0}}(\bar{p}^{c_{0}}_{b_{k+1}}-\bar{p}^{c_{0}}_{b_{k}}) \\
 -q^{b_{k}}_{c_{0}}& q^{b_{k}}_{c_{0}} &q^{b_{k}}_{c_{0}}\triangle t_{k}   & 0
\end{bmatrix}\begin{bmatrix}
v^{c_{0}}_{b_{k}}\\
v^{c_{0}}_{b_{k+!}}\\
g^{c_{0}}\\
s
\end{bmatrix}
\end{align}
\tag{3.14}
$$
在求取H矩阵的时候好像有点问题，式(3.12)带入式(3.14)中 $\alpha^{b_{k}}_{b_{k+1}}-q^{c_{0}}_{c_{k+1}}p^{c}_{b}+q^{c_{0}}_{c_{k}}p^{c}_{b}$ 后好像有点问题，以H(3,0)为例     
$$
\begin{align}\nonumber
&sq^{b_{k}}_{c_{0}}(\bar{p}^{c_{0}}_{b_{k+1}}-\bar{p}^{c_{0}}_{b_{k}}) \\\nonumber
&=q^{b_{k}}_{c_{0}}(s\bar{p}^{c_{0}}_{b_{k+1}}-q_{c_{k+1}}^{c_{0}}p^{b}_{c}-s\bar{p}^{c_{0}}_{b_{k}}+q_{c_{k}}^{c_{0}}p^{b}_{c}) \\\nonumber
&=q^{b_{k}}_{c_{0}}(s\bar{p}^{c_{0}}_{b_{k+1}}-s\bar{p}^{c_{0}}_{b_{k}}+q_{c_{k}}^{c_{0}}p^{b}_{c}-q_{c_{k+1}}^{c_{0}}p^{b}_{c}) \\\nonumber
&=q^{b_{k}}_{c_{0}}(s\bar{p}^{c_{0}}_{b_{k+1}}-s\bar{p}^{c_{0}}_{b_{k}}+q_{b_{k}}^{c_{0}}q_{c}^{b}p^{b}_{c}-q_{b_{k+1}}^{c_{0}}q_{c}^{b}p^{b}_{c}) \\\nonumber
&=q^{b_{k}}_{c_{0}}(s\bar{p}^{c_{0}}_{b_{k+1}}-s\bar{p}^{c_{0}}_{b_{k}})+q^{b_{k}}_{c_{0}}(q_{b_{k}}^{c_{0}}q_{c}^{b}p^{b}_{c}-q_{b_{k+1}}^{c_{0}}q_{c}^{b}p^{b}_{c})
\end{align}
$$
<font color=red>这样推的话，和(3.14)公式就对不上了啊。</font>算来算去是式(3.14)中z(0,0)的后两项差$q^{b_{k}}_{c_{0}}$ ,还是说推导出来的误差被(3.14)中的 $n_{b_{k+1}}^{b_{k}}$ 弥补了？但是这样的定性分析是怎么样的呢？    
最后求解最小二乘问题
$$
\underset{\delta b_{w}}{min}\sum_{k\in B}^{ }\left \|
\hat{z}^{b_{k}}_{b_{k+1}}-H^{b_{k}}_{b_{k+1}}X_{I}
 \right \|^{2}
$$
至此即可求解出所有状态量，但是对于重力向量 $g^{c_{0}}$ 还要做进一步的纠正。在纠正$g^{c_{0}}$ 的过程中，会对速度也做进一步的优化。      
#####3.3.3 纠正重力向量    
&emsp;&emsp;

###4 后端优化      
&emsp;&emsp;后端优化是VINS-Mono中除了初始化之外，创新性最高的一块，也是真真的 __紧耦合__ 部分，而初始化的过程事实上是一个 __松耦合__。因为初始化过程中的状态量并没有放在最底层融合，而是各自做了位姿的计算，但是在后端优化的过程中，所有优化量都是在一起的。      
&emsp;&emsp;状态量
$$
\begin{align}\nonumber
X&=[x_{0},x_{1},\cdots ,x_{n},x^{b}_{c},\lambda_{0},\lambda _{1}, \cdots , \lambda_{m}]  \\\nonumber
x_{k}&=[p^{w}_{b_{k}},v^{w}_{b_{k}},q^{w}_{b_{k}},b_{a},b_{g}],\quad k\in[0,n] \\\nonumber
x^{b}_{c}&= [p^{b}_{c},q^{b}_{c}]
\tag{4.1}
\end{align}
$$
优化过程中的误差状态量
$$
\begin{align}\nonumber
\delta X&=[\delta x_{0},\delta x_{1},\cdots ,\delta x_{n},\delta x^{b}_{c},\lambda_{0},\delta \lambda _{1}, \cdots , \delta \lambda_{m}]  \\\nonumber
\delta x_{k}&=[\delta p^{w}_{b_{k}},\delta v^{w}_{b_{k}},\delta \theta ^{w}_{b_{k}},\delta b_{a},\delta b_{g}],\quad k\in[0,n] \\\nonumber
\delta x^{b}_{c}&= [\delta p^{b}_{c},\delta q^{b}_{c}]
\end{align}
$$
进而得到系统优化的代价函数
$$\nonumber
\underset{X}{min}
\begin{Bmatrix}
\left \|
r_{p}-H_{p}X
\right \|^{2} +
\sum_{k\in B}^{ } \left \|
r_{B}(\hat{z}^{b_{k}}_{b_{k+1}},X)
\right \|^{2}_{P^{b_{k}}_{b{k+1}}} +
\sum_{(i,j)\in C}^{ } \left \|
r_{C}(\hat{z}^{c_{j}}_{l},X)
\right \|^{2}_{P^{c_{j}}_{l}}
\end{Bmatrix}
\tag{4.2}
$$
其中三个残差项依次是边缘化的先验信息、IMU测量残差以及视觉的观测残差。三种残差都是用马氏距离来表示的，这个在后面ceres的优化残差中要特别注意。     
&emsp;&emsp;在优化过程中，每一次的高斯迭代，式(4.2)可以进一步被线性化为，
$$
\underset{X}{min}
\begin{Bmatrix}
\left \|
r_{p}-H_{p}X
\right \|^{2} +
\sum_{k\in B}^{ } \left \|
r_{B}(\hat{z}^{b_{k}}_{b_{k+1}},X)+H^{b_{k}}_{b_{k+1}}\delta X
\right \|^{2}_{P^{b_{k}}_{b{k+1}}} +
\sum_{(i,j)\in C}^{ } \left \|
r_{C}(\hat{z}^{c_{j}}_{l},X)+H^{c_{j}}_{l}\delta X
\right \|^{2}_{P^{c_{j}}_{l}}
\end{Bmatrix}
\tag{4.3}
$$
其中 $H^{b_{k}}_{b_{k+1}}, H^{c_{j}}_{l}$ 为IMU测量和视觉测量方程的雅克比矩阵，在后面会有进一步的推导。最小化式(4.3)相当于求解线性方程
$$
(\Lambda _{p}+\Lambda _{B}+\Lambda _{C})\delta X=(b_{p}+b_{B}+b_{C})
\tag{4.4}
$$
其中 $\Lambda _{p}, \Lambda _{B}, \Lambda_{C}$ 分别对应边缘先验、IMU和视觉测量的信息矩阵。<font color=red>这部分还需要进一步的理解。</font>
####4.1 IMU测量误差      
&emsp;&emsp;这部分内容主要对应在后端的优化过程中IMU测量部分的残差以及在优化过程中的雅克比矩阵的求解。        
&emsp;&emsp;首先推导IMU测量的残差部分，由文献[[1]](#1)中body系下的预积分方程式(4)，可以得到IMU的测量模型式(13)
$$
\begin{bmatrix}
\hat{\alpha }^{b_{k}}_{b_{k+1}}\\
\hat{\gamma  }^{b_{k}}_{b_{k+1}}\\
\hat{\beta }^{b_{k}}_{b_{k+1}}\\
0\\
0
\end{bmatrix}
=\begin{bmatrix}
q^{b_{k}}_{w}(p^{w}_{b_{k+1}}-p_{b_{k}}^{w}+\frac{1}{2}g^{w}\triangle t^{2}-v_{b_{k}}^{w}\triangle t)\\
p_{b_{k}}^{w^{-1}}\otimes q^{w}_{b_{k+1}}\\
q^{b_{k}}_{w}(v^{w}_{b_{k+1}}+g^{w}\triangle t-v_{b_{k}}^{w})\\
b_{ab_{k+1}}-b_{ab_{k}}\\
b_{wb_{k+1}}-b_{wb_{k}}
\end{bmatrix}
\tag{4.3}
$$
那么IMU测量的残差即可写为
$$
\begin{align}\nonumber
r_{B}(\hat{z}^{b_{k}}_{b_{k+1}},X)=
\begin{bmatrix}
\delta \alpha ^{b_{k}}_{b_{k+1}}\\
\delta \theta   ^{b_{k}}_{b_{k+1}}\\
\delta \beta ^{b_{k}}_{b_{k+1}}\\
0\\
0
\end{bmatrix}
&=\begin{bmatrix}
q^{b_{k}}_{w}(p^{w}_{b_{k+1}}-p_{b_{k}}^{w}+\frac{1}{2}g^{w}\triangle t^{2}-v_{b_{k}}^{w}\triangle t)-\hat{\alpha }^{b_{k}}_{b_{k+1}}\\
[q_{b_{k+1}}^{w^{-1}}\otimes q^{w}_{b_{k}}\otimes \hat{\gamma  }^{b_{k}}_{b_{k+1}}]_{xyz}\\
q^{b_{k}}_{w}(v^{w}_{b_{k+1}}+g^{w}\triangle t-v_{b_{k}}^{w})-\hat{\beta }^{b_{k}}_{b_{k+1}}\\
b_{ab_{k+1}}-b_{ab_{k}}\\
b_{gb_{k+1}}-b_{gb_{k}}
\end{bmatrix}
\end{align}
\tag{4.4}
$$
其中 $[\hat{\alpha }^{b_{k}}_{b_{k+1}},\hat{\gamma  }^{b_{k}}_{b_{k+1}},\hat{\beta }^{b_{k}}_{b_{k+1}}]$ 来自于IMU预积分部分。式(4.4)和文献[1]中的式(22)有些不同，主要是第1项，在两个式子中是逆的关系，本文中的写法是为了和代码保持一致。本质上两者是没有区别的，因为残差很小的时候，第一项接近于单位四元数，所以取逆并没有什么影响，只是写残差的方式不一样。   
&emsp;&emsp;~~在式(4.4)中，残差主要来自于两帧IMU的位姿、速度及Bias，即 $[p^{w},q^{w},v^{w},b_{a},b_{w}]$，或者说我们要利用IMU残差要优化的状态量也是这5个，如果是两帧IMU就是10个。在计算雅克比矩阵的时候，分为  $[p^{w}_{b_{k}},q^{w}_{b_{k}}]$ , $[v^{w}_{b_{k}},b_{ab_{k}},b_{wb_{k}}]$，$[p^{w}_{b_{k+1}},q^{w}_{b_{k+1}}]$ , $[v^{w}_{b_{k}},b_{ab_{k}},b_{wb_{k}}]$ 等四部分， 分别表示为 $J[0],J[1],J[2],J[3]$。这部分内容的推导可以参考文献`[2]` 中III.B式(14)部分。~~     
&emsp;&emsp;这个地方很容易出错误，原因就是没有搞清楚求偏导的对象。整个优化过程中，IMU测量模型这一块，涉及到的状态量是 $x_{k}$,但是参考文献`[5]`中IV.A部分，以及十四讲中10.2.2小节的目标函数可知，这个地方的雅克比矩阵是针对变化量 $\delta x_{k}$的。所以，在后面求取四部分雅可比的时候，也不是对状态量求偏导，而是对误差状态量求偏导。
&emsp;&emsp;式(4.4)对 $[\delta p^{w}_{b_{k}},\delta \theta ^{w}_{b_{k}}]$ 求偏导得，
$$
J[0]=\begin{bmatrix}
-q^{b_{k}}_{w} & R^{b_{k}}_{w}[(p^{w}_{b_{k+1}}-p_{b_{k}}^{w}+\frac{1}{2}g^{w}\triangle t^{2}-v_{b_{k}}^{w}\triangle t)]_{\times }\\
0 & [q_{b_{k+1}}^{w^{-1}}q^{w}_{b_{k}}]_{L}[\hat{\gamma  }^{b_{k}}_{b_{k+1}}]_{R}J^{\gamma}_{b_{w}}\\
0 & R^{b_{k}}_{w}[(v^{w}_{b_{k+1}}+g^{w}\triangle t-v_{b_{k}}^{w})]_{\times } \\
0 &0
\end{bmatrix}
\tag{4.5}
$$
 $J[0]$ 是 $15*7$的矩阵，其中 $R^{b_{k}}_{w}$ 是四元数对应的旋转矩阵，求偏导可以看做是对四元数或者旋转矩阵求偏导，求解过程可以参考文献[[2]](#2)。第3项只需右下角的3行3列。第1项可以参考文献`[1]`中4.3.4“旋转向量的雅克比矩阵”推导得到。<font color=red> 其中第3项是按照代码写出的，不是很理解。</font>      
 &emsp;&emsp;式(4.4)对 $[\delta v^{w}_{b_{k}},\delta b_{ab_{k}},\delta b_{wb_{k}}]$ 求偏导得，
 $$J[1]=
 \begin{bmatrix}
-q^{b_{k}}_{w}\triangle t & -J^{\alpha }_{b_{a}} & -J^{\alpha }_{b_{a}}\\
0 & 0 & -[q_{b_{k+1}}^{w^{-1}}\otimes q^{w}_{b_{k}}\otimes \hat{\gamma  }^{b_{k}}_{b_{k+1}}]_{L}J^{\gamma}_{b_{w}}\\
-q^{b_{k}}_{w} & -J^{\beta }_{b_{a}} & -J^{\beta }_{b_{a}}\\
 0& -I &0 \\
0 &0  &-I
\end{bmatrix}
\tag{4.6}
 $$
 $J[1]$ 是 $15*9$的矩阵，第5项仍然取右下角的3行3列。    
  &emsp;&emsp;式(4.4)对 $[\delta p^{w}_{b_{k+1}},\delta \theta ^{w}_{b_{k+1}}]$ 求偏导得，
$$J[2]=
\begin{bmatrix}
-q^{b_{k}}_{w} &0\\
0 &  [\hat{\gamma  }^{b_{k}^{-1}}_{b_{k+1}}\otimes q_{w}^{b_{k}}\otimes q_{b_{k+1}}^{w}]_{L} \\
0 & 0 \\
 0& 0  \\
0 &0   
\end{bmatrix}
\tag{4.7}
$$
$J[2]$ 是 $15*9$的矩阵，第5项仍然取右下角的3行3列。   
  &emsp;&emsp;式(4.4)对 $[\delta v^{w}_{b_{k}},\delta b_{ab_{k}},\delta b_{wb_{k}}]$ 求偏导得，
$$J[3]=
\begin{bmatrix}
-q^{b_{k}}_{w} &0 & 0\\
0 & 0 &0 \\
q^{b_{k}}_{w} & 0 & 0\\
 0& I &0 \\
0 &0  &I
\end{bmatrix}
\tag{4.8}
$$
$J[3]$ 是 $15*9$的矩阵。        
以上便是IMU测量模型的雅可比的矩阵的推导过程，雅克比矩阵主要还是在Ceres做优化的过程中，高斯迭代要使用到。
####4.2 相机测量误差      
&emsp;&emsp;相机测量误差万变不离其宗，还是要会到像素坐标差或者灰度差(光度误差)。Vins-Mono中的相机测量误差本质还是特征点的重投影误差，将特征点 $P$从相机的$i$ 系转到相机的$j$ 系，即把相机的测量残差定义为，
$$
r_{C}=(\hat{z}_{l}^{c_{j}},X)=[b_{1},b_{2}]^{T}\cdot (\bar{P}_{l}^{c_{j}}-\frac{P_{l}^{c_{j}}}{\left \| P_{l}^{c_{j}} \right \|})
\tag{4.9}
$$
因为最终要将残差投影到切平面上，$[b_{1},b_{2}]$是切平面上的一对正交基。反投影后的$P_{l}^{c_{j}}$ 写为，
$$
P_{l}^{c_{j}}=q_{b}^{c}(q_{w}^{b_{j}}(q_{b_{i}}^{w}(q_{c}^{b} \frac{\bar{P}_{l}^{c_{i}}}{\lambda _{l}}+p_{c}^{b})+p_{b_{i}}^{w}-p_{b_{j}}^{w})-p_{c}^{b})
$$
反投影之前的坐标 $\bar{P}_{l}^{c_{j}}$,$\bar{P}_{l}^{c_{i}}$写为，
$$
\begin{align}\nonumber
\bar{P}_{l}^{c_{j}}&=\pi_{c}^{-1}(\begin{bmatrix}
\hat{u}_{l}^{c_{j}}\\ 
\hat{v}_{l}^{c_{j}}
\end{bmatrix}) \\\nonumber
\bar{P}_{l}^{c_{i}}&=\pi_{c}^{-1}(\begin{bmatrix}
\hat{u}_{l}^{c_{i}}\\ 
\hat{v}_{l}^{c_{i}}
\end{bmatrix})
\end{align}
$$
&emsp;&emsp;参与相机测量残差的状态量有，$[p^{w}_{b_{i}},q^{w}_{b_{i}}]$，$[p^{w}_{b_{j}},q^{w}_{b_{j}}]$，$[p^{b}_{c},q^{b}_{c}]$以及逆深度 $\lambda_{l}$。所以下面分别对这几个状态量对应的误差状态量求式(4.9)的偏导，得到高斯迭代过程中的雅克比矩阵。
&emsp;&emsp;式(4.9)对$[\delta p^{w}_{b_{i}},\delta \theta ^{w}_{b_{i}}]$求偏导，得到$3*6$的雅克比矩阵，
$$
J[0]=\begin{bmatrix}
q_{b}^{c}q_{w}^{b_{j}} & -q_{b}^{c}q_{w}^{b_{j}}q_{b_{i}}^{w}[q_{c}^{b} \frac{\bar{P}_{l}^{c_{i}}}{\lambda _{l}}+p_{c}^{b}]_{\times }
\end{bmatrix}
\tag{4.10}
$$
&emsp;&emsp;式(4.9)对$[\delta p^{w}_{b_{j}},\delta \theta ^{w}_{b_{j}}]$求偏导，得到$3*6$的雅克比矩阵，
$$
J[1]=\begin{bmatrix}
-q_{b}^{c}q_{w}^{b_{j}} & q_{b}^{c}q_{w}^{b_{j}}[q_{b_{i}}^{w}(q_{c}^{b} \frac{\bar{P}_{l}^{c_{i}}}{\lambda _{l}}+p_{c}^{b})+p_{b_{i}}^{w}-p_{b_{j}}^{w}]_{\times }
\end{bmatrix}
\tag{4.11}
$$
&emsp;&emsp;式(4.9)对$[\delta p^{b}_{c},\delta \theta ^{b}_{c}]$求偏导，得到$3*6$的雅克比矩阵，
$$
J[2]=
\begin{bmatrix}
q_{b}^{c}(q_{w}^{b_{j}}q_{bi}^{w}-I_{3*3}) & -q_{b}^{c}q_{w}^{b_{j}}q_{b_{i}}^{w}q_{c}^{b}[\frac{\bar{P}_{l}^{c_{i}}}{\lambda _{l}}]_{\times }+[q_{b}^{c}(q_{w}^{b_{j}}(q_{b_{i}}^{w}p_{c}^{b}+p_{b_{i}}^{w}-p_{b_{j}}^{w})-p_{c}^{b})]
\end{bmatrix}
\tag{4.12}
$$
&emsp;&emsp;式(4.9)对$\delta \lambda_{l}$求偏导，得到$3*1$的雅克比矩阵，
$$
J[3]=-q_{b}^{c}q_{w}^{b_{j}}q_{b_{i}}^{w}q_{c}^{b} \frac{\bar{P}_{l}^{c_{i}}}{\lambda _{l}^{2}}
\tag{4.13}
$$
以上就是相机测量误差以及误差方程的雅克比矩阵的求解过程。
&emsp;&emsp;<span style="color:red;">在相机测量误差的文件`projection_factor.cpp`中的`ProjectionFactor::check`还没有理解</span>
####4.3 闭环修正与优化       
&emsp;&emsp;__这部分内容默认已经检测到闭环，只涉及到后续的优化部分。__ 
####4.4 系统边缘化
&emsp;&emsp;在利用Sliding Window做优化的过程中，滑窗内的pose和feature个数是有限的，在系统优化的过程中，势必要不断将一些pose和feature移除滑窗，但是无故地移除这些pose和feature会丢弃某些约束，进而降低了优化的精度，所以在移除pose和feature的时候需要将相关联的约束转变为一个先验的约束条件作为prior放到优化问题中，以上就可以描述为marginalization(边缘化)的过程。    
###5 闭环校正
&emsp;&emsp;直接法不能像特征法那样直接将拿描述子利用Dbow进行闭环检测，所以在闭环的时候只能重新对Keyframe提取Brief等描述子建立Database进行闭环检测，或者使用开源的闭环检测库__[FabMap](https://github.com/arrenglover/openfabmap)__来进行闭环检测。Vins-Mono选择了第一种闭环检测的方法。      
####5.1 闭环检测
&emsp;&emsp;Vins-Mono还是利用词袋的形式来做Keyframe Database的构建和查询。在建立闭环检测的数据库时，关键帧的Features包括两部分：VIO部分的200个强角点和500 Fast角点。然后描述子仍然使用BRIEF(因为旋转可观，匹配过程中对旋转有一定的适应性，所以不用使用ORB)。   
####5.2 闭环校正
&emsp;&emsp;在闭环检测成功之后，会得到回环候选帧。所以要在已知位姿的回环候选帧和滑窗内的匹配帧做匹配，然后把回环帧加入到滑窗的优化当中，这时整个滑窗的状态量的维度是不发生变化的，因为回环帧的位姿是固定的。
###6 全局的位姿优化      
&emsp;&emsp;因为之前做的非线性优化本质只是在一个滑窗之内求解出了相机的位姿，而且在回环检测部分，利用固定位姿的回环帧只是纠正了滑窗内的相机位姿，并没有修正其他位姿(或者说没有将回环发现的误差分配到整个相机的轨迹上)，缺少全局的一致性，所以要做一次全局的Pose Graph。__全局的Pose Graph较之滑窗有一定的迟滞性__，只有相机的Pose滑出滑窗的时候，Pose才会被加到全局的Pose Graph当中。

****
###7 关键变量含义和代码注释中的一些称呼
&emsp;&emsp;因为VINS-Mono的代码写的有点乱，好多变量的命名过于随意，不能很好理解，而且为了注释方便自己也给某些变量或者环节做了命名。
####7.1 代码注释中的自定义名称
- 闭环帧和匹配帧
&emsp;&emsp;闭环帧：在闭环匹配成功之后，Keyframe Database中的旧帧;匹配帧：当前滑窗之内，和闭环帧匹配的匹配帧。

####7.1 代码注释中的自定义名称
- [image(estimator.cpp)](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator.cpp#L108)
&emsp;&emsp;某一帧图像得到的某个特征点，某一帧图像得到的特征点，`<Feature_id, <camera_id,Feature>>`
- Headers
&emsp;&emsp;一般指图像的帧头
- f_manager
&emsp;&emsp;滑窗的特征点管理器
- [keyframe_database(estimator_node.cpp)](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator_node.cpp#L54)
&emsp;&emsp;关键帧集合，每次在闭环检测线程中增加新元素，每个关键帧在滑窗内的导数第二个位置时被[添加](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator_node.cpp#L243)。
- [keyframe_buf(estimator_node.cpp)](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator_node.cpp#L29)
&emsp;&emsp;为闭环检测提供临时的Keyframe存储,[只包含滑窗内的倒数第二Keyframe](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator_node.cpp#L522)，当其包含的某帧进入闭环检测环节会被[弹出](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator_node.cpp#L236)。
- [retrive_data_vector](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator.h#L130)
&emsp;&emsp;
****