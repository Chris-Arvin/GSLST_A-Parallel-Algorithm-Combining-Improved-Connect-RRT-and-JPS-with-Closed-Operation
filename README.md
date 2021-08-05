# GSLST_A-Parallel-Algorithm-Combining-Improved-Connect-RRT-and-JPS-with-Closed-Operation
Global Sampling and Local Search Tree(GSLST), a parallel algorithm combining Connect-RRT and JPS with Closed-Operation.  

The related paper named "A Parallel Algorithm Combining Connect-RRT and JPS with Closed-Operation" is accepted by CASE2020.

后记：笔者后来查看这段代码时，发现由于线程间没有使用线程锁，所以有些时候该算法的计算时间会显著增大。
note：When review this code, I found that because the thread lock is not correctly used between threads, sometimes the calculation time of the algorithm will increase significantly.
