---
layout: post
title: "MVG Chapter 2-Projective Geometry and Transformations of 2D"
date:       2018-10-24
author:     Tong
catalog: true
tags:
    - MVG
---

## 2.1 Planar geometry
1. What is _Tensor_? 

2. A significant advantage of the algebraic approach to geometry is that results derived in this way may more easily be used to derive algorithms and practical computational methods.

## 2.2 The 2D projective plane
### 2.2.1 Points and lines

3. __Homogeneous representation of lines.__ $$ ax + by + c = 0 $$ and $$ (ka)x + (kb)y + (kc) = 0 $$ are the same, for any non-zero constant $$k$$. Thus, the vectors $$ (a, b, c)^\intercal $$ and $$ k(a, b, c)^\intercal $$ represent the same line, for any non-zero $$k$$.

4. __Homogeneous representation of points.__ $$ (x,y,1)^\intercal $$.

5. __Result 2.1.__ _The points $$ x $$ lies on the line $$ l $$ if and only if $$ x^\intercal l = 0 $$._

6. __Degrees of freedom (dof).__ A line is specified by two parameters (the two independent ratios $$ \big\{a : b : c\big\} $$) and so has two degrees of freedom.

7. __Result 2.2.__ _The intersection of two lines $$ l $$ and $$ l^\prime $$ is the point $$ x =  l \times {l^\prime} $$._

8. __Result 2.4.__ _The line through two points $$ x $$ and $$ x^\prime $$ is the point $$ l =  x \times x^\prime $$._

### 2.2.2 Ideal points and the line at infinity

9. __Intersection of parallel lines.__ E.g. $$ ax+by+c = 0 $$ and $$ ax+by+c^\prime = 0 $$, represented by vectors $$ l = (a,b,c)^\intercal $$ and $$ l^\prime = (a,b,c^\prime)^\intercal $$. The intersection is $$ l \times l^\prime = (c^\prime - c)(b,-a,0)^\intercal $$, and igoring the scale factor $$ (c^\prime - c) $$, this is the point $$ (b,-a,0)^\intercal $$.

10. __Ideal points and the line at infinity.__ Homogeneous vectors $$ x = (x_1,x_2,x_3)^\intercal $$ such that $$ x_3 \neq 0 $$ correspond to finite points in $$ \mathbb{R^2} $$. One may augment $$ \mathbb{R^2} $$ by adding points with last coordinate $$x_3 = 0$$. The resulting space is the set of all homogeneous 3-vectors, namely the projective space $$ \mathbb{P^2} $$. The points with last coordinate $$ x_3 = 0 $$ are known as _ideal_ points, or points at infinity. The set of all ideal points may be written $$ (x_1,x_2,0)^\intercal $$, with a particular point specified by the ratio $$x_1:x_2$$. Note this set lies on a single line, the _line of infinity_, denoted by the vector $$l_\infty = (0,0,1)^\intercal$$. The line at inifinity can be also thought of as the set of directions of lines in the plane.

11. __A model for the projective plane.__ A fruitful way of thinking of $$ \mathbb{P^2} $$ is as a set of rays in $$ \mathbb{R^3} $$. <br>
![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-ray_model.png)
*Fig. A model of the projective plane*
	
12. __Reuslt 2.6. Duality principle.__ _To any theorem of 2-dimensional projective geometry there corresponds a dual theorem, which may be derived by interchanging the roles of points and lines in the original theorem._ Example: Result 2.2 and 2.4.

### 2.2.3 Conics and dual conics

13. The euqation of a conic in inhomogeneous coordinate is
$$
ax^2 + bxy + cy^2 + dx + ey + f = 0
$$ 
<br> i.e. a polynomial of degree 2. "Homogenizing" this by the replacements: $$ x \mapsto x_1/x_3, y \mapsto x_2/x_3 $$ gives
$$
a{x_1}^2 + b{x_1}{x_2} + c{x_2}^2 + d{x_1}{x_3} + e{x_2}{x_3} + f{x_3}^2 = 0 \quad (2.1) 
$$
<br> or in matrix form <br>
$$
x^\intercal C x = 0 \quad (2.2) 
$$
<br>where the conic coefficient matrix $$C$$ is given by <br>
$$
C = \begin{bmatrix} a & b/2 & d/2 \\ b/2 & c & e/2 \\ d/2 & e/2 & f \end{bmatrix} \quad (2.3) 
$$
<br> Note that the conic coefficient matrix is symmetric. C is a homogeneous representation of a conic. The conic has five degrees of freedom which can be thought of as the ratios $$ \big\{a : b : c : d ： e : f\big\} $$ or equivalently the six elemnts of a symmetric matrix less one for scale.

14. __Five points define a conic.__ Stacking the constraints from five points we obtain, where $$c=(a,b,c,d,e,f)^\intercal$$<br>
$$
\begin{bmatrix} {x_1}^2 & x_1 y_1 & {y_1}^2 & x_1 & y_1 & 1 \\ {x_2}^2 & x_2 y_2 & {y_2}^2 & x_2 & y_2 & 1 \\ {x_3}^2 & x_3 y_3 & {y_3}^2 & x_3 & y_3 & 1 \\ {x_4}^2 & x_4 y_4 & {y_4}^2 & x_4 & y_4 & 1 \\ {x_5}^2 & x_5 y_5 & {y_5}^2 & x_5 & y_5 & 1\end{bmatrix} c = 0 \quad (2.4) 
$$

15. __Result 2.7__ _The line $$ l $$ tangent to $$ C $$ at a point $$ x $$ on $$ C $$ is given by $$ l = Cx $$._

16. __Dual conics.__ The conic $$C$$ defined above is more properly termed a _point_ conic, as it defines an equation on points. Given the duality result 2.6 of $$\mathbb{P^2} $$ it is not surprising that there is also a conic which defines an equation on lines. This _dual_ (or line) conic is also represented by a $$3 \times 3$$ matrix, which we denote as $$C^*$$. A line $$l$$ _tangent_ to the conic $$C$$ satisfies $$l^\intercal C^* l = 0$$. The notation $$C^*$$ indicates that $$C^*$$ is the adjoint matrix of $$C$$. For a non-singular symmetric matrix $$C^*=C^{-1}$$.
<br>Note: The equation for a dual conic is straightforward to derive in the case that $$C$$ has full rank.
![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-line_conic.png)
*Fig. Dual conics are also known as conic envelopes.*

17. __Degenerate conics.__ If the matrix $$C$$ is not of full rank, then the conic is termed degenerate. Degenerate point conics include two lines (rank 2), and a repeated line (rank 1). E.g. $$C = l m^\intercal + m l^\intercal$$.
	
## 2.3 Projective transformations

18. __Defition 2.9.__ A _projectivity_ is an invertible mapping _h_ from $$ \mathbb{ P^2 } $$ to itself such that three points $$x_1$$, $$x_2$$ and $$x_3$$ lie on the same line if and only if $$h(x_1), h(x_2), h(x_3)$$do.
<br> Projectivities form a group since the inverse of a projectivity is also a projectivity, and so is the composition of two projectivities. A projectivity is also called a _collineation_, a _projective transformation_ or a _homography_.

19. __Theorem 2.10.__ _A mapping h : $$ \mathbb{ P^2 } \rightarrow \mathbb{ P^2 } $$ is a projectivity if and only if there exists a non-singular 3 $$ \times $$ 3 matrix $$ \mathbf{H} $$ such that for any point in $$ \mathbb{ P^2 } $$ represented by a vector $$x$$ it is true that $$ h(x) = Hx $$_.
<br> To interpret this theorem, any point in $$\mathbb{P^2}$$ is represented as a homogeneous 3-vector, $$x$$, and $$Hx$$ is a linear mapping of homogeneous coordinates.
<br> __Proof.__ Let $$x_1,x_2$$ and $$x_3$$ lie on a line $$l$$. Thus $$l^\intercal x_i = 0$$ for $$i = 1,...,3$$. Let $$H$$ be a non-singular $$3 \times 3$$ matrix. One verifies that $$l^\intercal H^{-1} H x_i = 0$$. Thus, the points $$H x_i$$ all lie on the line $$ H^{-\intercal} l $$, and collinearity is preserved by the transformation.

20. __Definition 2.11. Projective transformation.__ A planar projective transformation is a linear transformation on homogeneous 3-vectors represented by a non-singular $$3 \times 3$$ matrix:<br>
$$
\begin{pmatrix} x^\prime_1 \\ x^\prime_2 \\ x^\prime_3 \end{pmatrix} = \begin{bmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{bmatrix} \begin{pmatrix} x_1 \\ x_2 \\ x_3 \end{pmatrix} \quad (2.5) 
$$
<br>or more briefly, $$x^\prime = Hx$$
<br>Note that the matrix $$H$$ in this equation may be changed by multiplication by an arbitrary non-zero sclae factor without altering the projective transformation. Consequently we say that $$H$$ is a _homography_ matrix, since as in the homogeneous representation of a point, only the ratio of the matrix elements is significant. There are eight independent ratios amongst the nine elements of $$H$$, and it follows that a projective transformation has eight degrees of freedom.
<br>A projective transformation projects every figure into a projectively equivalent figure, leaving all its projective properties invariant.

21. __Mappings between planes.__ Projection along rays through a common point (the centre of projection) defines a mapping from one plane to another. If a coordinate system is defined in each plane and points are represented in homogeneous coordinates, then the _central projection_ mapping may be expressed by $$x^\prime = Hx$$ where $$H$$ is a non-singular $$3 \times 3$$ matrix.
<br> Actually, if the two coordinate systems defined in the two planes are both Euclidean (rectilinear) coordinate systems then the mapping defined by central projection is more restricted than an arbitrary projective transformation. It is called _perspectivity_ rather than a full projectivity, and may be represented by a transformation with six degrees of freedom.

### 2.3.1 Transformations of lines and conics

22. __Transformation of lines.__ According to the proof of theorem 2.10, under the point transformation $$x^\prime = Hx$$, a line transforms as 
$$
l^\prime = H^{-\intercal} l  \quad (2.6)
$$
<br> One may alternatively write $$l^{\prime \intercal} = l^\intercal H^{-1}$$. Note the fundamentally different way in which lines and points transform. Points transform according to $$H$$, whereas lines (as rows) transform according to $$H^{-1}$$. This may be explained in terms of "covariant" or "contravariant" behaviour. One says that points transform _contravariantly_ and lines transform _covariantly_.

23. __Transformation of conics.__ Under a point transformation $$x^\prime = Hx$$, (2.2) becomes
$$
\begin{align}
x^\intercal C x & = x^{\prime \intercal}[H^{-1}]^\intercal C H^{-1} x^\prime  \\
                & = x^{\prime \intercal}H^{-\intercal} C H^{-1} x^\prime
\end{align}
$$
<br> which is a quadratic form $$x^{\prime T} C^\prime x\prime$$ with $$C^\prime = H^{-\intercal} C H^{-1}$$

24. __Result 2.13.__ _Under a point transformation $$ x^\prime = Hx $$, a conic $$ C $$ transforms to $$ C^\prime = H^{-\intercal}CH^{-1} $$._
<br> The presence of $$H^{-1}$$ in this transformation may be expressed by saying that a conic transforms _covariantly_.

25. __Result 2.14.__ _Under a point transformation x^\prime = Hx, a dual conic $$ C^* $$ transforms to $$ C^{*\prime} = HC^*H^\intercal $$._

## 2.4 A hierarchy of transformations

26. The group of invertible  $$n \times n$$ matrices with real elements is the (real) _general linear group_ on $$n$$ dimensions, or $$GL(n)$$. To obtain the _projective linear group_ the matrices related by a scalar multiplier are identified, giving $$PL(n)$$ (this is a quotient group of $$GL(n)$$). In the case of projective transformations of the plane $$n = 3$$.
<br> The important subgroups of $$PL(3)$$ include _affine group_, which is the subgroup of $$PL(3)$$ consisting of matrices for which the last row is (0, 0, 1), 
<br> and the _Euclidean group_, which is a subgroup of the affine group for which in addition the upper left hand $$2 \times 2$$ matrix is orthogonal. 
<br> One may also identify the _oriented Euclidean group_ in which the upper left hand $$2 \times 2$$ matrix has determinant 1.

27. __Invariants.__ An alternative to describing the transformation _algebraically_, i.e. as a matrix acting on coordiantes of a point or curve, is to describe the transformation in terms of those elements or quantities that are preserved or _invariant_.
<br>Note: a Euclidean transformation (translation and rotation). A similarity (e.g. translation, rotation and isotropic scaling).

### 2.4.1 Class I: Isometries

28. Isometries are transformations of the plane $$\mathbb{R^2}$$ that preserve Euclidean distance (from _iso_ = same, _metric_ = measure). An isometry is represented as
$$
\begin{pmatrix} x^\prime \\ y^\prime \\ 1 \end{pmatrix} = \begin{bmatrix} \epsilon \cos \theta & - \sin \theta & t_x \\ \epsilon \sin \theta & \cos \theta & t_y \\ 0 & 0 & 1 \end{bmatrix} \begin{pmatrix} x \\ y \\ 1 \end{pmatrix} 
$$
<br> where $$\epsilon = \pm 1$$. If $$\epsilon = 1$$ then the isometry is _orientation-preserving_ and is a _Euclidean_ transformation (a composition of a translation and rotation). If $$\epsilon = -1$$ then the isometry reverses orientation. An example is the composition of a reflection.
<br> Euclidean transformations model the motion of a rigid object. However, the orientation reversing isometries often arise as ambiguities in structure recovery.
<br> A planar Euclidean transformation can be written more concisely in block form as 
$$
x^\prime = H_E x = \begin{bmatrix} R & t \\ 0^\intercal & 1 \end{bmatrix} x  \quad (2.7)
$$
<br> where $$R$$ is a $$2 \times 2$$ rotation matrix (an orthogonal matrix such that $$R^\intercal R = R R^\intercal = I$$), $$t$$ a translation 2-vector, and $$0$$ a null 2-vector. 
<br> A Euclidean transformation is also known as a _displacement_.
<br> A planar Euclidean transformation has three degrees of freedom, one for the rotation and two for the translation. Thus three parameters must be specified in order to define the transformation. The transformation can be computed from two point correspondences (because one point can provide two equations).

29. __Invariants.__ Length (the distance between two points), angle (the angle between two lines), and area.

30. __Groups and orientation.__ An isometry is orientation-preserving if the upper left hand $$2 \times 2$$ matrix has determinant 1. Orientation-_preserving_ isometries form a group, orientation-_reversing_ ones do not.

### 2.4.2 Class II: Similarity transformations

31. A similarity transformation (or more simply a _similarity_) is an isometry composed with an isotropic scaling. In the case of a Euclidean transformation composed with a scaling (i.e. no reflection) the similarity has matrix representation
$$
\begin{pmatrix} x^\prime \\ y^\prime \\ 1 \end{pmatrix} = \begin{bmatrix} s \cos \theta & - s \sin \theta & t_x \\ s \sin \theta & s \cos \theta & t_y \\ 0 & 0 & 1 \end{bmatrix} \begin{pmatrix} x \\ y \\ 1 \end{pmatrix} \quad (2.8)
$$
<br> This can be written more concisely in block form as 
$$
x^\prime = H_S x = \begin{bmatrix} sR & t \\ 0^\intercal & 1 \end{bmatrix} x  \quad (2.9)
$$
<br> where the scalar _s_ represents the isotropic scaling. A similarity transformation is also known as an _equi-form_ transformation, because it preserves "shape"(form). A planar similarity transformation has four degrees of freedom, the scaling accounting for one more degree of freedom tham a Euclidean transformation. A similarity can be computed from two point correspondences.

32. __Invariants.__ Angles between lines are not affected by rotation, translation or isotropic scaling, and so are similarity invariants. In particular parallel lines are mapped to parallel lines. The length between two points is not a similarity invariant, but the _ratio_ of two lengths/areas is an invariant.

33. __Metric structure.__ The description _metric structure_ implies that the structure is defined up to a similarity.

### 2.4.3 Class III: Affine transformation

34. An affine transformation (or more simply an _affinity_) is a non-singular linear transformation followed by a translation. It has the matrix representation
$$
\begin{pmatrix} x^\prime \\ y^\prime \\ 1 \end{pmatrix} = \begin{bmatrix} a_{11} & a_{12} & t_x \\ a_{21} & a_{22} & t_y \\ 0 & 0 & 1 \end{bmatrix} \begin{pmatrix} x \\ y \\ 1 \end{pmatrix} \quad (2.10)
$$
<br> or in block form
$$
x^\prime = H_A x = \begin{bmatrix} A & t \\ 0^\intercal & 1 \end{bmatrix} x  \quad (2.11)
$$
<br> with $$A$$ a $$2 \times 2$$ non-singular. A planar affine transformation has six degrees of freedom corresponding to the six matrix elements. The transformation can be computed from three point correspondences.
<br> A helpful way to understand the geometric effects of the linear component $$A$$ of an affine transformation is as the composition of two fundamental transformations, namely rotations and non-isotropic scalings. The affine matrix $$A$$ can always be decomposed as 
$$
A = R(\theta) R(-\phi) D R(\phi) \quad (2.12)
$$
<br> where $$R(\theta)$$ and $$R(\phi)$$ are rotations by $$\theta$$ and $$\phi$$ respectively, and $$D$$ is a diagonal matrix:
$$
D = \begin{bmatrix} \lambda_1 & 0 \\ 0 & \lambda_2 \end{bmatrix}
$$
<br> This decomposition follows directly from the SVD.
<br> The affine matrix $$A$$ is hence seen to be the concatenation of a rotation (by $$\phi$$); a scaling by $$\lambda_1$$ and $$\lambda_2$$ respectively in the (rotated) $$x$$ and $$y$$ directions; a rotation back (by $$-\phi$$); and a finally another rotation (by $$\theta$$). The only "new" geometry, compared to a similarity, is the non-isotropic scaling. This accounts for the two extra degrees of freedom possessed by an affinity over a similarity. They are the angle $$\phi$$ specifying the scaling direction, and the ratio of the scaling parameters $$\lambda_1 : \lambda_2$$. 
<br> The essence of an affinity is this scaling in orthogonal directions, oriented at a particular angle.
![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-affine_def_all.png)
*Fig. Distortions arising from a planar affine transformation*

35. __Invariants.__ Because an affine transformation includes non-isotropic scaling, the similarity invariants of length ratios and angles between lines are not preserved under an affinity. Three important invariants are:
	- __Parallel lines.__ 
	- __Ratio of lengths of parallel line segments.__
	- __Ratio of areas.__
	
36. An affinity is orientation-preserving or -reversing according to whether $$\det A$$ is positive or negative respectively. Since $$\det A = \lambda_1 \lambda_2$$ the property depends only on the sign of the scalings.

### 2.4.4 Class IV: Projective transformations

37. A projective transformation was defined in (2.5). It is a general non-singular linear transformation of _homogeneous_ coordiantes. This generalizes an affine transformation, which is the composition of a general non-singular linear transformation of _inhomogeneous_ coordinates and a translation. Here we examine its block form
$$
x^\prime = H_P x = \begin{bmatrix} A & t \\ \mathbf{v}^\intercal & \upsilon \end{bmatrix} x  \quad (2.13)
$$
<br> where the vector $$\mathbf{v} = (\upsilon_1,\upsilon_2)^\intercal$$. The matrix has nine elements with only their ratio significant, so the transformation is specified by eight parameters. Note, it is not always possible to sclae the matrix such that $$\upsilon$$ is unity since $$\upsilon$$ might be zero. A projective transformation between two planes can be computed from four point correspondences, with no three collinear on either plane.
<br> Unlike the case of affinity, it is not possible to distinguish between orientation preserving and orientation reversing projectivities in $$\mathbb{P^2}$$. 

38. __Invariants.__ The most fundamental projective invariant is the cross ratio of four collinear points: a ratio of lengths on a line is invariant under affinities, but not under projectivities. However, a ratio of ratios or _cross ratio_ of lengths on a line is a projective invariant.

### 2.4.5 Summary and comparison

39. The key difference between a projective and affine transformation is that the vector $$\mathbf{v}$$ is not null for a projectivity.This is responsible for the non-linear effects of the projectivity. Compare the mapping of an ideal point $$(x_1,x_2,0)^\intercal$$ under an affinity and projectivity: 
<br> First the affine transformation
$$
\begin{bmatrix} A & t \\ 0^\intercal & 1 \end{bmatrix} \begin{pmatrix} x_1 \\ x_2 \\ 0 \end{pmatrix} = \begin{pmatrix} A \begin{pmatrix} x_1 \\ x_2 \end{pmatrix}  \\ 0 \end{pmatrix}   \quad (2.14)
$$
<br> Second the projective transformation
$$
\begin{bmatrix} A & t \\ \mathbf{v}^\intercal & \upsilon \end{bmatrix} \begin{pmatrix} x_1 \\ x_2 \\ 0 \end{pmatrix} = \begin{pmatrix} A \begin{pmatrix} x_1 \\ x_2 \end{pmatrix}  \\ \upsilon_1 x_1 + \upsilon_2 x_2 \end{pmatrix} \quad (2.15)
$$
<br> In the first case the ideal point remains ideal (i.e. at infinity). In the second it is mapped to a finite point. It is this ability which allows a projective transformation to model vanishing points.

### 2.4.6 Decomposition of a projective transformation

40. A projective transformation can be decomposed into a chain of transformations, where each matrix in the chain represents a transformation higher in the hierarchy than the previous one.
$$
H = H_S H_A H_P = \begin{bmatrix} sR & t \\ 0^\intercal & 1 \end{bmatrix}  \begin{bmatrix} K & 0 \\ 0^\intercal & 1 \end{bmatrix} \begin{bmatrix} I & 0 \\ \mathbf{v}^\intercal & \upsilon \end{bmatrix}  = \begin{bmatrix} A & t \\ \mathbf{v}^\intercal & \upsilon \end{bmatrix}   \quad (2.16)
$$
<br> with $$A$$ a non-singular matrix given by $$A = sRK + t \mathbf{v}^\intercal$$, and $$K$$ an upper-triangular matrix normalized as $$\det K = 1$$. This decomposition is valid provided $$\upsilon \neq 0$$, and is unique if $$s$$ is chosen positive.
<br> Each of the matrices $$H_S, H_A, H_P$$ is the "essence" of a transformation of that type. Consider the process of rectifying the perspective image of a plane: $$H_P$$ (2 dof) moves the line at infinity; $$H_A$$ (2 dof) affects the affine properties, but does not move the line at infinity; and finally, $$H_S$$ is a general similarity transformation (4 dof) which does not affect the affine or projective properties. The transformation $$H_P$$ is an _elation_.
<br> This decomposition can be employed when the objective is to only partially determine the transformation. For example, if one wants to measure length ratios from the perspective image of a plane, then it is only neccessary to determine (rectify) the transformation up to similarity.

41. Taking the inverse of $$H$$ in (2.16) gives $$H^{-1} = H_P^{-1} H_A^{-1} H_S^{-1}$$. Since $$H_P^{-1}, H_A^{-1}$$ and $$H_S^{-1}$$ are still projective, affine and similarity transformations respectively, a general projective transformation may also be decomposed in the form
$$
H = H_S H_A H_P = \begin{bmatrix} I & 0 \\ \mathbf{v}^\intercal & \upsilon \end{bmatrix}  \begin{bmatrix} K & 0 \\ 0^\intercal & 1 \end{bmatrix}  \begin{bmatrix} sR & t \\ 0^\intercal & 1 \end{bmatrix}  \quad (2.17)
$$ 
<br> Note that the actual values of $$K， T， t$$ and $$\mathbf{v}$$ will be different from those of (2.16).

### 2.4.7 The number of invariants

42. __Result 2.16.__ _The number of functionally independent invariants is equal to, or greater than, the number of degrees of freedom of the configuration less the number of degrees of freedom of the transformation._
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-geometric-properties-invariant.PNG)
*Tab. Geometric properties invariant to commonly occurring _planar_ transformations.*

## 2.5 The projective geometry of 1D

43. The development of the projective geometry of a line, $$\mathbb{P^1}$$, proceeds in much the same way as that of the plane. A point $$x$$ on the line is represented by homogeneous coordiantes $$(x_1,x_2)^\intercal$$, and a point for which $$x_2 = 0$$ is an ideal point of the line. We will use the notation $$\overline{x}$$ to represent the 2-vector $$(x_1,x_2)^\intercal$$. A projective transformation of a line is represented by a $$2 \times 2$$ homogeneous matrix,
$$
\overline{x}^\prime = H_{2 \times 2} \overline{x}
$$
<br> and has 3 degrees of freedom corresponding to the four elements of the matrix less one for overall scaling. A projective transformation of a line may be determined from three corresponding points.

44. __The cross ratio.__ The cross ratio is the basic projective invariant of $$\mathbb{P^1}$$. Given 4 points $$\overline{x_i}$$ the _cross ratio_ is defined as <br>
$$
Cross(\overline{x_1},\overline{x_2},\overline{x_3},\overline{x_4}) = \frac{\mid\overline{x_1}\overline{x_2}\mid \mid\overline{x_3}\overline{x_4}\mid}{\mid\overline{x_1}\overline{x_3}\mid \mid\overline{x_2}\overline{x_4}\mid}
$$
<br> where $$\mid\overline{x_i}\overline{x_j}\mid = \det \begin{bmatrix} x_{i1} & x_{j1} \\ x_{i2} & x_{j2} \end{bmatrix}$$
<br> A few comments on the cross ratio.
	- The value of the cross ratio is not dependent on which particular homogeneous representative of a point $$\overline{x_i}$$ is used, since the scale cancels between numerator and denominator.
	- If each point $$\overline{x_i}$$ is a finite point and the homogeneous representative is chosen such that $$x_2 = 1$$, then $$\mid\overline{x_i}\overline{x_j}\mid$$ represents the signed distance from $$\overline{x_i}$$ to $$\overline{x_j}$$.
	- The definition of the cross ratio is also valid if one of the points $$\overline{x_i}$$ is an ideal point.
	- The value of the cross ratio is invariant under any projective transformation of the line: if $$\overline{x^\intercal} = H_{2 \times 2} \overline{x}$$ then <br>
	$$
		Cross(\overline{x_1}^\prime,\overline{x_2}^\prime,\overline{x_3}^\prime,\overline{x_4}^\prime) = Cross(\overline{x_1},\overline{x_2},\overline{x_3},\overline{x_4})   \quad (2.18)
	$$

45. __Concurrent lines.__ A configuration of concurrent lines is dual to collinear points on a line. This means that concurrent lines on a plane also have the geometry $$\mathbb{P^1}$$. In particular four concurrent lines have a cross ratio as illustrated in next figure.
![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-concurrent_lines.png)
*Fig. Concurrent lines.*

![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-line_camera.png)
*Fig. Projection of points in $$\mathbb{P^2}$$ into a 1-dimensional image.*

## 2.6 Topology of the projective plane

## 2.7 Recovery of affine and metric properties from images

46. A projective transformation has only 4 degrees of freedom more than a similarity, so it is only neccessary to specify 4 degrees of freedom (not 8) in order to determine metric properties. In projective geometry these 4 degrees of freedom are given "physical substance" by being associated with geometric objects: the line at infinity $$l_\infty$$ (2 dof), and the two _circular points_ (2 dof) on $$l_\infty$$.

47. In the following it is shown that the projective distortion may be removed once the image of $$l_\infty$$ is specified, and the affine distortion removed once the image of the circular points is specified. Then the only remaining distortion is a similarity.

### 2.7.1  The line at inifinity

48. Under a projective transformation ideal points may be mapped to finite points (2.15), and consequently $$l_\infty$$ is mapped to a finite line. However, if the transformation is an affinity, then $$l_\infty$$ is not mapped to a finite line, but remains at inifinity. According to the line transformation (2.6): <br>
$$
l_\infty^\prime = H_A^{-\intercal} l_\infty = \begin{bmatrix} A^{-\intercal} & 0 \\ -t^\intercal A^{-\intercal} & 1 \end{bmatrix} \begin{pmatrix} 0 \\ 0 \\ 1\end{pmatrix} = \begin{pmatrix} 0 \\ 0 \\ 1\end{pmatrix} = l_\infty
$$
<br> An affine transformation is the most general linear transformation that fixes $$l_\infty$$.

49. __Result 2.17.__ _The line at infinity, $$l_\infty$$, is a fixed line under the projective transformation $$H$$ if and only if $$H$$ is an affinity._
<br> However, $$l_\infty$$ is not fixed pointwise under an affine transformation: (2.14) showed that under an affinity a point on $$l_\infty$$ (an ideal point) is mapped to a point on $$l_\infty$$, but it is not the same point undless $$A(x_1,x_2)^\intercal = k(x_1,x_2)^\intercal$$. 

### 2.7.2 Recovery of affine properties from images

50. Once the imaged line at inifinity is identified in an image of a plane, it is then possible to make affine measurements on the original plane. For example, lines may be identified as parallel on the original plane if the images lines intersect on the imaged $$l_\infty$$. This follows because parallel lines  on the Euclidean plane on $$l_\infty$$, and after a projective transformation the lines still intersect on the images $$l_\infty$$ since intersections are preserved by projectivities.

51. Similarly, once $$l_\infty$$ is identified a length ratio on a line may be computed from the cross ratio of the three points specifying the lengths together with the intersection of the line with $$l_\infty$$ (which provides the fourth point for the cross ratio).

52. However, a less tortuous path which is better suited to computational algorithms is simply to transform the identified $$l_\infty$$ to its canonical position of $$l_\infty = (0,0,1)^\intercal$$. The (projective） matrix which achieves this transformation can be applied to every point in the image in order to affinely rectify the image, i.e. after the transformation, affine measurements can be made directly from the rectified image. The key idea here is in the following figure:
![](https://raw.githubusercontent.com/TongLing916/tongling916.github.io/master/img/post-Affine-rectification-via-the-vanishing-line.PNG)

53. If the imaged line at inifinity is the line $$ l = (l_1, l_2, l_3)^\intercal $$, then provided $$l_3 \neq 0$$ a suitable projective point transformation which will map $$l$$ back to $$l_\infty = (0,0,1)^\intercal$$ is
$$
H = H_A \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ l_1 & l_2 & l_3 \end{bmatrix}  \quad (2.19)
$$ 
<br> where $$H_A$$ is any affine transformation (the last row of $$H$$ is $$l^\intercal$$). One can verify that under the line transformation (2.6): <br>
$$
H^{-\intercal} (l_1,l_2,l_3)^\intercal = (0,0,1)^\intercal = l_\infty
$$

54. __Example 2.18. Affine rectification__ <br>
In a perspective image of a plane, the line at infinity on the world plane is imaged as the vanishing line of the plane. The vanishing line $$l$$ may be computed by intersecting imaged parallel lines. The image is then rectified by applying a projective warping (2.19) such that $$l$$ is mapped to its canonical position $$l_\infty = (0,0,1)^\intercal$$
<br> This example shows that affine properties may be recovered by simply specifying a line (2 dof).

55. __Example 2.19. Computing a vanishing point from a length ratio__ <br>
Given two intervals on a line with a known length ratio, the point at infinity on the line may be determined. A typical case is where three points $$ a^\prime, b^\prime, c^\prime$$ are identified on a line in an image. Suppose $$a, b$$ and $$c$$ are the corresponding collinear points on the world line, and the length ratio $$d(a,b) : d(b,c) = a:b$$ is known (where $$d(x,y)$$ is the Euclidean distance between the points $$x$$ and $$y$$). It is possible to find the vanishing point using the cross ratio. 
<br>	(1) Measure the distance ratio in the image, $$ d(a^\prime, b^\prime):d(b^\prime,c^\prime) = a^\prime:b^\prime$$.
<br>	(2) Points $$a, b$$ and $$c$$ may be represented as coordinates $$0, a$$ and $$a+b$$ in a coordinate frame on the line $$<a,b,c>$$. For computational purposes, these points are represented by homogeneous 2-vectors $$(0,1)^\intercal, (a,1)^\intercal$$ and $$(a+b,1)^\intercal$$. Similarly, $$a^\prime, b^\prime$$ and $$c^\prime$$ have coordinates $$0, a^\prime$$ and $$a^\prime + b^\prime$$, which may also be expressed as homogeneous vectors.
<br>	(3) Relative to these coordiantes frames, compute the 1D projective transformation $$H_{2 \times 2}$$ mapping $$ a \mapsto a^\prime, b \mapsto b^\prime$$ and $$c \mapsto c^\prime$$.
<br>	(4) The image of the point at inifinity (with coordinates $$(1,0)^\intercal$$) under $$H_{2 \times 2}$$ is the vanishing point on the line $$<a^\prime,b^\prime,c^\prime>$$.
	
56. __Example 2.20. Geometric construction of vanishing points from a length ratio.__ <br>
The vanishing points may also be computed by a purely geometric construction consisting of the following steps:
<br> (1) Given: three collinear points, $$a^\prime, b^\prime$$ and $$c^\prime$$, in an image corresponding to collinear world points with interval ratio $$a:b$$.
<br> (2) Draw any line $$l$$ through $$a^\prime$$ (not coincident with the line $$a^\prime c^\prime$$), and mark off points $$a = a^\prime, b$$ and $$c$$ such that the line segments $$<ab>, <bc>$$ have length ratio $$a:b$$.
<br> (3) Join $$b b^\prime$$ and $$c c^\prime$$ and intersect in $$o$$.
<br> (4) The line through $$o$$ parallel to $$l$$ meets the line $$a^\prime c^\prime$$ in the vanishing point $$v^\prime$$.
![](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/WebPage/pngfiles/projgeomfigs-construction_lengthratio.png)
*Fig. A geometric construction to determine the image of the point at infinity on a line given a known length ratio.*

### 2.7.3 The circular points and their dual

57. Under any similarity transformation there are two points on $$l_\infty$$ which are fixed. These are the _circular points_ (also called the _absolute points_) $$I, J$$, with canonical coordinates <br>
$$
I = \begin{pmatrix} 1 \\ \mathrm{i} \\ 0 \end{pmatrix} \quad J = \begin{pmatrix} 1 \\ \mathrm{-i} \\ 0 \end{pmatrix} 
$$
<br> The circular points are a pair of complex conjugate ideal points. To see that they are fixed under an orientation-preserving similarity: <br>
$$
\begin{align}
I^\prime & = H_S I  \\
         & = \begin{bmatrix} s \cos \theta & -s \sin \theta & t_x \\ s \sin \theta & s \cos \theta & t_y \\ 0 & 0 & 1 \end{bmatrix} \begin{pmatrix} 1 \\ \mathrm{i} \\ 0 \end{pmatrix} \\
		 & = s e^{-\mathrm{i} \theta} \begin{pmatrix} 1 \\ \mathrm{i} \\ 0 \end{pmatrix} = I
\end{align}
$$
<br> with an analogous proof for $$J$$. A reflection swaps $$I$$ and $$J$$. The converse is also true, i.e. if the circular points are fixed then the linear transformation is a similarity.

58. __Result 2.21.__ _The circular points, $$I$$, $$J$$, are fixed points under the projective transformation $$H$$ if and only if $$H$$ is a similarity._
<br> The name "circular points" arises because every circle intertersects $$l_\infty$$ at the circular points.

59. Algebraically, the circular points are the orthogonal directions of Euclidean geometry, $$(1,0,0)^\intercal$$ and $$(0,1,0)^\intercal$$, packaged into a single complex conjugate entity, e.g. <br>
$$
I = (1,0,0)^\intercal + \mathrm{i}(0,1,0)^\intercal.
$$
<br> Consequently, it is not surprising that once the circular points are identified, orthogonality, and other metric properties, are then determined.

60. __The conic dual to the circular points.__ The conic <br>
$$
C_\infty^{*} = I J^\intercal + J I^\intercal \quad (2.20)
$$
<br> is dual to the circular points.The conic $$C_\infty^{*}$$ is a degenerate (rank 2) line conic (see section 2.2.3), which consists of the two points. In a Euclidean coordiante system it is given by <br>
$$
C_\infty^{*} = \begin{pmatrix} 1 \\ \mathrm{i} \\ 0 \end{pmatrix} \begin{pmatrix} 1 & \mathrm{-i} & 0 \end{pmatrix} + \begin{pmatrix} 1 \\ \mathrm{-i} \\ 0 \end{pmatrix} \begin{pmatrix} 1 & \mathrm{i} & 0 \end{pmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\0 & 0 & 0 \end{bmatrix}
$$

61. The conic $$C_\infty^{*}$$ is fixed under similarity transformations in an analogous fashion to the fixed properties of circular points. A conic is fixed if the same matrix results (up to scale) under the transformation rules . Since $$C_\infty^{*}$$ is a dual conic it transforms according to result 2.14 ($$C^{* \prime} = H C^{\prime} H^\intercal $$), and one can verify that under the point transformation $$x^\prime = H_S x$$, <br>
$$
{C_\infty^{*}}^\prime = H_S C_\infty^{*} H_S^\intercal = C_\infty^{*}
$$
<br> The converse is also true.

62. __Result 2.22.__ _The dual conic $$C_\infty^{*}$$ is fixed under the projective transformation H if and only if H is a similarity._
<br> Some properties of $$C_\infty^{*}$$ in any projective frame:
	- $$C_\infty^{*}$$ has 4 degrees of freedom.
	- $$l_\infty$$ is the null vector of $$C_\infty^{*}$$. I.e. $$C_\infty^{*} l_\infty = 0$$

### 2.7.4 Angles on the projective plane

63. For the lines $$l = (l_1,l_2,l_3)^\intercal$$ and $$m = (m_1,m_2,m_3)^\T$$ with normals parallel to $$(l_1,l_2)^\intercal, (m_1,m_2)^\intercal$$ respectively, the angle is <br>
$$
\cos \theta = \frac{l_1 m_1 + l_2 m_2}{\sqrt{(l_1^2 + l_2^2)(m_1^2 + m_2^2)}}  \quad (2.21)
$$
<br> The problem with this expression is that the first two components of $$l$$ and $$m$$ do not have well defined transformation properties under projective transformations (they are not tensors), and so (2.21) cannot be applied after an affine or projective transformation of the plane. However, an analogous expression to (2.21) which is invariant to projective transformation is <br>
$$
\cos \theta = \frac{l^\intercal C_\infty^{*} m}{\sqrt{(l^\intercal C_\infty^{*} l)(m^\intercal C_\infty^{*} m)}}  \quad (2.22)
$$

64. __Result 2.23.__ _Once the conic $$C_\infty^{*}$$ is identified on the projective plane then Euclidean angles may be measured by (2.22)._

65. __Result 2.24.__ _Lines $$l$$ and $$m$$ are orthogonal if $$l^\intercal C_\infty^{*} m$$._

66. __Length ratios__ may also be measured once $$C_\infty^{*}$$ is identified.

### 2.7.5 Recovery of metric properties from images

67. __Metric rectification using $$C_\infty^{*}$$.__ The dual conic $$C_\infty^{*}$$ neatly packages all the information required for a metric rectification. It enables both the projective and affine components of a projective transformation to be determined, leaving only similarity distortions. If the point transformation is $$x^\prime = Hx$$, where the $$x$$-coordinate frame is Euclidean and $$x^\prime$$ projective, $$C_\infty^{*}$$ transforms according to result 2.14 ($$C^{* \prime} = H C^* H^\prime $$). Using the decomposition chain (2.17):<br>
$$
\begin{align}
{C_\infty^*}^\prime & = (H_P H_A H_S) C_\infty^* (H_P H_A H_S)^\intercal = (H_P H_A) (H_S C_\infty^* H_S^\intercal) (H_A^\intercal H_P^\intercal) \\
					& = (H_P H_A) C_\infty^* (H_A^\intercal H_P^\intercal) \\
					& = \begin{bmatrix} KK^\intercal & KK^\intercal v \\ v^\intercal K K^\intercal &  v^\intercal K K^\intercal v\end{bmatrix} \quad (2.23)
\end{align}
$$

68. __Result 2.25.__ _Once the conic $$C_\infty^*$$ is identified on the projective plane then projective distortion may be rectified up to a similarity._
<br> Actually, a suitable rectifying homography may be obtained directly from the identified $$C_\infty^{*\prime}$$ in an image using the SVD （section A4.4): writing the SVD of $$C_\infty^{*\prime}$$ as <br>
$$
C_\infty^{*\prime} = U \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 0 \end{bmatrix} U^\intercal
$$
<br> then by inspection from (2.23) the rectifying projectivity is $$H = U$$ up to a similarity.

69. __Stratification.__ First the projective and subsequently the affine distortions were removed. This two-approach is termed _stratified_.

## 2.8 More properties of conics

### 2.8.1 The pole-polar relationship

70. A point $$x$$ and a conic $$C$$ define a line $$l = Cx$$. The line $$l$$ is called the _polar_ of $$x$$ with respect to $$C$$, and the point $$x$$ is the _pole_ of $$l$$ with respect to C.

71. _The polar line $$l = Cx$$ of the point $$x$$ with respect to a conic $$C$$ intersects the conic in two points. The two lines tangent to $$C$$ at these points intersect at $$x$$.

72. _If the point $$x$$ is on $$C$$ then the polar is the tangent line to the conic at $$x$$._

73. __Definition 2.29.__ A _correlation_ is an invertible mapping from points of $$\mathbb{P^2}$$ to lines of $$\mathbb{P^2}$$. It is represented by a $$3 \times 3$$ non-singular matrix $$A$$ as $$l = Ax$$.

74. __Conjugate points.__ _If the point y is on the line $$l = Cx$$ then $$y^\intercal l = y^\intercal Cx = 0$$. Any two points $$x, y$$ satisfying $$y^\intercal Cx = 0$$ are conjugate with respect to the conic C._

75. _If $$x$$ is on the polar of $$y$$ then $$y$$ is on the polar of x._ <br> This follows simply because of the symmetry of the conic matrix.

### 2.8.2 Classification of conics

76. __Projective normal form for a conic__.

77. __Affine classification of conics.__

## 2.9 Fixed points and lines

78. The key idea is that an _eigenvector_ corresponds to a _fixed point_ of the transformation, since for an eigenvector $$e$$ with eigenvalue $$\lambda$$,<br>
$$
He = \lambda e
$$
<br> and $$e$$ and $$\lambda e$$ represent the same point.

79. __A Euclidean matrix.__ The two ideal fixed points are the complex conjugate pair of circular points $$I, J$$, with corresponding eigenvalues $$\big\{e^{i \theta}, e^{-i \theta}\big\}$$, where $$\theta$$ is the rotation angle. The third eigenvector, which has unit eigenvalue, is called the _pole_. The Euclidean transformation is equal to a pure rotation by $$\theta$$ about this point with no translation.
<br> A special case is that of a pure translation (i.e. where $$\theta = 0$$). Here the eigenvlaues are triply degenerate.

80. __A similarity matrix.__ The two ideal fixed points are again the circular points. The eigenvalues are $$\big\{1, se^{i \theta}, se^{-i \theta}\big\}$$. The action can be understood as a rotation and isotropic scaling by $$s$$ about the finite fixed point.

81. __An affine matrix.__ The two ideal fixed points can be real or complex conjugates, but the fixed line $$ l_\infty = (0,0,1)^\intercal$$ through these points is real in either case.

## 2.10 Closure
### 2.10.1 The literature

### 2.10.2 Notes and exercises

1. __Affine transformations.__ <br>
	
2. __Projective transformations.__ <br>

3. __Isotropies.__ <br>

4. __Invariants.__ <br>

5. __The cross ratio.__ <br>

6. __Polarity.__ <br>

7. __Conics.__ <br>

8. __Dual conics.__ <br>

9. __Special projective transformations.__ <br>
































