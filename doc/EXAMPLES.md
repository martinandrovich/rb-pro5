# Examples

This document contains examples of project relevant problems with the use of various methods from the repository.

---

## K-means clustering algorithm

A paragraph that describes the problem statement and which methods are used to solve it.

```cpp
// load img
auto img_src = utils::loadimg("filepath/to/img.jpg");

// perform k-mean with k = 8; output is new mat
auto img_k_mean = utils::k_mean(img_src, 8);
```

---
