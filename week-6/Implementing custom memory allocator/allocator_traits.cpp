template<typename T>
struct pointer_traits {
  using reference = T &;
  using const_reference = const T &;
};

// Avoid declaring a reference to void with an empty specialization
template<>
struct pointer_traits<void> {
};

template<typename T = void>
struct MyAllocator : public pointer_traits<T> {
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;
  using difference_type = typename std::pointer_traits<pointer>::difference_type;

  MyAllocator() noexcept;

  ~MyAllocator() noexcept;

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept;

  T * allocate(size_t size, const void * = 0);

  void deallocate(T * ptr, size_t size);

  template<typename U>
  struct rebind {
    typedef MyAllocator<U> other;
  };
};

template<typename T, typename U>
constexpr bool operator==(const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept;

template<typename T, typename U>
constexpr bool operator!=(const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept;