## 1 预备知识  
通过DH建模，可计算出笛卡尔空间中，从机器人基坐标原点到末端法兰中心点的位姿矩阵，记为 $_{E}^{B}T$。  
从末端法兰中心点到工具尖端点，也可以通过一个位姿矩阵来描述，记为${}_T^ET$。由于该两点间并没有自由度变量，所以对于某一确定工具的确定尖端，${}_T^ET$总是定值常量。  
根据机器人位姿变换规则，从机器人基坐标原点到工具尖端点的位姿矩阵，可由矩阵${}_E^BT$右乘矩阵${}_T^ET$得到，记为${}_T^BT$。
$${}_E^BT*{}_T^ET = {}_T^BT$$
工具标定算法的实质，就是要求出矩阵${}_T^ET$。对于4x4的矩阵${}_T^ET$，可分块表示为：  
$$
{}_T^ET = \left[
    \begin{matrix}
    {}_T^ER & {}_T^EP \\
    0 & 1
    \end{matrix}
    \right]
$$
其中，${}_T^ER$为3x3矩阵，表示姿态偏移转换；${}_T^EP$为3x1矩阵，表示位置偏移转换。  

## 2 标定操作  
进行6点工具标定时，前3点位置（XYZ）相同，需满足工具目标尖端与某一固定点刚好碰上；姿态（ABC）任意，需满足尽可能姿态差别明显。后3点姿态（ABC）相同，位置（XYZ）不同，若第4点垂直接触于固定点；第5点在第4点的基础上，另工具尖端的位置偏移到期望工具坐标系的X+方向；第6点在第4点的基础上，另工具尖端的位置偏移到期望工具坐标系的Z+方向。  

## 3 位姿解算  
### 3.1 位置偏移解算  
$$
\begin{matrix}
\because
{}_E^B{T_i}*{}_T^ET = {}_T^B{T_i}\\

\therefore \left[ {\begin{matrix}
{{}_E^B{R_i}}&{{}_E^B{P_i}}\\
0&1
\end{matrix}} \right]*\left[ {\begin{matrix}
{{}_T^ER}&{{}_T^EP}\\
0&1
\end{matrix}} \right] = \left[ {\begin{matrix}
{{}_T^B{R_i}}&{{}_T^B{P_i}}\\
0&1
\end{matrix}} \right]\\

\therefore {}_E^B{R_i}*{}_T^EP + {}_E^B{P_i} = {}_T^B{P_i}
\end{matrix}
$$
由于前4点的位置（XYZ）相同：
$$
\begin{matrix}
{}_T^B{P_{\left( {i = 1..4} \right)}} = {}_E^B{R_1}*{}_T^EP + {}_E^B{P_1}\\
 = {}_E^B{R_2}*{}_T^EP + {}_E^B{P_2}\\
 = {}_E^B{R_3}*{}_T^EP + {}_E^B{P_3}\\
 = {}_E^B{R_4}*{}_T^EP + {}_E^B{P_4}
\end{matrix}
$$

移项可得：
$$
\left\{ {\begin{matrix}
{{}_E^B{P_1} - {}_E^B{P_2} = \left( {{}_E^B{R_2} - {}_E^B{R_1}} \right)*{}_T^EP}\\
{{}_E^B{P_2} - {}_E^B{P_3} = \left( {{}_E^B{R_3} - {}_E^B{R_2}} \right)*{}_T^EP}\\
{{}_E^B{P_3} - {}_E^B{P_4} = \left( {{}_E^B{R_4} - {}_E^B{R_3}} \right)*{}_T^EP}
\end{matrix}} \right.
$$

改写成矩阵形式可得：
$$
{}_T^EP = {\left[ {\begin{matrix}
{{}_E^B{R_2} - {}_E^B{R_1}}\\
{{}_E^B{R_3} - {}_E^B{R_2}}\\
{{}_E^B{R_4} - {}_E^B{R_3}}
\end{matrix}} \right]^ + }*\left[ {\begin{matrix}
{{}_E^B{P_1} - {}_E^B{P_2}}\\
{{}_E^B{P_2} - {}_E^B{P_3}}\\
{{}_E^B{P_3} - {}_E^B{P_4}}
\end{matrix}} \right]\\

\therefore
{}_T^EP = {\left( {{{\left[ {\begin{matrix}
{{}_E^B{R_2} - {}_E^B{R_1}}\\
{{}_E^B{R_3} - {}_E^B{R_2}}\\
{{}_E^B{R_4} - {}_E^B{R_3}}
\end{matrix}} \right]}^T}*\left[ {\begin{matrix}
{{}_E^B{R_2} - {}_E^B{R_1}}\\
{{}_E^B{R_3} - {}_E^B{R_2}}\\
{{}_E^B{R_4} - {}_E^B{R_3}}
\end{matrix}} \right]} \right)^{ - 1}}*{\left[ {\begin{matrix}
{{}_E^B{R_2} - {}_E^B{R_1}}\\
{{}_E^B{R_3} - {}_E^B{R_2}}\\
{{}_E^B{R_4} - {}_E^B{R_3}}
\end{matrix}} \right]^T}*\left[ {\begin{matrix}
{{}_E^B{P_1} - {}_E^B{P_2}}\\
{{}_E^B{P_2} - {}_E^B{P_3}}\\
{{}_E^B{P_3} - {}_E^B{P_4}}
\end{matrix}} \right]
$$
将前4点位姿带入上式，即可求得位置偏移${}_T^EP$。

### 3.2 姿态偏移解算  

$$
\left\{ {\begin{matrix}
{{}_T^BN = \frac{{{}_T^B{P_5} - {}_T^B{P_4}}}{{\left| {{}_T^B{P_5} - {}_T^B{P_4}} \right|}}}\\
{{}_T^BA = \frac{{{}_T^B{P_6} - {}_T^B{P_4}}}{{\left| {{}_T^B{P_6} - {}_T^B{P_4}} \right|}}}
\end{matrix}} \right.
$$

由第4点和第5点确定工具坐标系的X+方向向量${}_T^BN$  
由第4点和第6点确定工具坐标系的Z+方向向量${}_T^BA$  

$$
{}_T^BO = {}_T^BA \times {}_T^BN\\
{}_T^BA = {}_T^BN \times {}_T^BO
$$

再根据XYZ三个方向向量，两两正交，${}_T^BA$与${}_T^BN$进行矩阵叉乘，可求得Y+方向向量${}_T^BO$。  
因此，可求得从机器人基坐标原点到工具尖端点的位姿矩阵为：
$$
{}_T^BR = \left[ {\begin{matrix}
{{}_T^BN}&{{}_T^BO}&{{}_T^BA}
\end{matrix}} \right]
$$

根据后3点位姿相同，求得从末端法兰中心点到工具尖端点的姿态偏移矩阵${}_T^ER$：
$$
\because {}_E^B{R_4} = {}_E^B{R_5} = {}_E^B{R_6}\\
\therefore {}_T^BR = {}_E^B{R_4}*{}_T^ER = {}_E^B{R_5}*{}_T^ER = {}_E^B{R_6}*{}_T^ER\\
\therefore {}_T^ER = {}_E^B{R_4}^{ - 1}*{}_T^BR
$$

### 3.3 位姿偏移解算
最终，可组合出从末端法兰中心点到工具尖端点的完整位姿偏移矩阵${}_T^ET$：
$$
{}_T^ET = \left[
    \begin{matrix}
    {}_T^ER & {}_T^EP \\
    0 & 1
    \end{matrix}
    \right]
$$

## 参考文献
[1]熊烁,叶伯生,蒋明.机器人工具坐标系标定算法研究[J].机械与电子,2012(06):60-63.
