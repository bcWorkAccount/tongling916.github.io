---
layout: post
title: "Chapter 4: Estimation-2D Projective Transformations"
date:       2018-11-09
author:     Tong
catalog: true
tags:
    - MVG
---

### 4.1 The Direct Linear Transformation (DLT) algorithm

1. $$\begin{bmatrix}0^\intercal & -\omega _i^\prime x_i^\intercal & y_i^\prime x_i^\intercal\\ \omega _i^\prime x_i^\intercal & 0^\intercal & -x_i^\prime x_i^\intercal\\ -y_i^\prime x_i^\intercal & x_i^\prime x_i^\intercal & 0^\intercal \end{bmatrix} \begin{pmatrix}h^1\\ h^2 \\ h^3 \end{pmatrix} = 0    \quad \quad (4.1)$$

2. $$h = \begin{pmatrix}
h^1\\ h^2 \\ h^3
\end{pmatrix}, H = \begin{bmatrix}
h_1 & h_2 & h_3 \\ 
h_4 & h_5 & h_6 \\ 
h_7 & h_8 & h_9
\end{bmatrix}    \quad \quad$$ (4.2)

> Algorithm 4.1. The basic DLT for H (but see algorithm 4.2 which includes normalization <br>
> $$ \underline{Objective} $$ <br>
> Given $$n \geq 4$$ 2D to 2D point correspondences $$\left \{ x_i \leftrightarrow x_i^\prime \right \}$$, determine the 2D homography matrix H such that $$x_i^\prime = Hx_i$$.<br>
> $$ \underline{Algorithm} $$ <br>
> (i) For each correspondence $$ x_i \leftrightarrow x_i^\prime $$ compute the matrix $$A_i$$ from(4.1). Only the first two rows end need be used in general.   <br>
> (ii) Assemble the $$n \: 2 \times 9$$ matrices $$A_i$$ into a single $$2n \times 9$$ matrix A.<br>
> (iii) Obtain the SVD of A. The unit singular vector corresponding to the smallest singular value is the solution $$h$$. Sepcifically, if $$A = UDV^\intercal$$ with $$D$$ diagonal with positive diagonal entries, arranged in descending order down the diagonal, then $$h$$ is the last colomn of $$V$$.<br>
> (iv) The matrix $$H$$ is determined from $$h$$ as in (4.2).

3. A situation where a configuration does not determine a unique solution for a particular class of transformation is termed _degenerate_.


### 4.2 Different cost functions

### 4.3 Statistical cost functions and Maximum Likelihood estimation

### 4.4 Transformation invariance and normalization

### 4.5 Iterative minimization methods

### 4.6 Experimental comparison of the algorithms

### 4.7 Robust estimation 

### 4.8 Automatic computation of a homography

 
### 4.9 Closure

































