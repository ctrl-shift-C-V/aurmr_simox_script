#include <exception>
#include <iterator>
#include <vector>

namespace simox::iterator {

// FIXME: there is a bug. The last element will not be visited.


/** The NestedRangeIterator class
*
*   The purpose of this class is to provide an iterator over a nested vector e.g. vector<vector<...>> .
*   This iterator can then be passed to all std algorithms.
*/
template <typename ElementType>
class NestedRangeIterator
    : public std::iterator<std::input_iterator_tag, ElementType> {
public:
  using InnerRangeType = std::vector<ElementType>;
  using NestedRangeType = std::vector<InnerRangeType>;

  typedef typename InnerRangeType::iterator RangeIterator;
  typedef typename NestedRangeType::iterator OuterRangeIterator;

  NestedRangeIterator() = delete;

  NestedRangeIterator(NestedRangeType &nested_range)
      : NestedRangeIterator(nested_range, nested_range.begin(),
                            nested_range.begin()->begin()) {}

  NestedRangeIterator(NestedRangeType &nested_range,
                      OuterRangeIterator it_outer, RangeIterator it_inner)
      : nested_range(nested_range), it_outer(it_outer), it_inner(it_inner) {}

  NestedRangeIterator &operator++() {

      if(endReached()){
          return *this;
      }

    // first increment inner iterator if possible
    // never set to end as it is invalid
    if (it_inner != it_outer->end()) {
      ++it_inner;
    }

    if ((it_inner != it_outer->end())) {

      assert(not endReached());
      return *this;
    }

    // we reached end of inner loop
    // increment outer iterator
    if (it_outer != nested_range.end()) {
      ++it_outer;

      if (it_outer != nested_range.end()) {
        // now set inner loop iterator to the beginning of that range
        it_inner = it_outer->begin();
      } else {
        std::cout << "End reached" << std::endl;
      }

      return *this;
    }

    return *this;
  }

  ElementType &operator*() { return *it_inner; }

  const ElementType &operator*() const { return *it_inner; }

  bool operator==(const NestedRangeIterator &other) const {
    return (it_inner == other.it_inner) and (it_outer == other.it_outer);
  }

  bool operator!=(const NestedRangeIterator &other) const {
    return not operator==(other);
  }

  NestedRangeIterator begin() { return NestedRangeIterator(nested_range); }

  //! the end is defined as: outer iterator valid, inner invalid
  NestedRangeIterator end() {

    const auto &valid_outer = nested_range.end() - 1;

    return NestedRangeIterator(nested_range, valid_outer, valid_outer->end()-1);
  }

  bool endReached(){
      const auto &valid_outer = nested_range.end() - 1;
      const auto& end_inner = valid_outer->end();
      
      return (valid_outer == it_outer) and (end_inner == it_inner);

    // return NestedRangeIterator(nested_range, valid_outer, valid_outer->end());
  }

private:
  NestedRangeType &nested_range;

  OuterRangeIterator it_outer;
  RangeIterator it_inner;
};

} // namespace simox::iterator