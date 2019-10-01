# Utilities

The `utils.h` module contains various utility centered methods for easy of use throghout the project; these methods are categorized by their respective fields.

---

## Computer vision

### `utils::loadimg()`

Load an image image given a from a filepath; the methods returns a `cv::Mat` if the given filepath is valid. An optional `cv::ImreadModes` flag can be passed as a second argument.

```cpp
std::optional<cv::Mat>
utils::loadimg(const std::string& filepath, cv::ImreadModes cv_flags = cv::IMREAD_UNCHANGED);
```

### `utils::another_one()`

Lorem ipsum dolor sit amet, consectetur adipiscing elit. Vestibulum sed porta mauris. Quisque euismod quam at magna accumsan porta. Etiam tristique ex lacus, at varius est lacinia quis.

```cpp
;
```

---
