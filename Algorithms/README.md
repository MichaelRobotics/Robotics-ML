# Sorting Algorithms

This repository contains Python implementations of various sorting algorithms. Each algorithm is implemented as a standalone Python file that you can run directly to see the algorithm in action.

## Algorithms Included

- **Bubble Sort** (`bubbleSort.py`): A simple comparison-based algorithm that repeatedly steps through the list, compares adjacent elements, and swaps them if they are in the wrong order.
  - Time Complexity: O(n²) in worst and average cases, O(n) in best case
  - Space Complexity: O(1)

- **Counting Sort** (`countingSort.py`): A non-comparative integer sorting algorithm that operates by counting the number of objects that possess distinct key values.
  - Time Complexity: O(n+k) where k is the range of the input
  - Space Complexity: O(n+k)

- **Heap Sort** (`HeapSort.py`): A comparison-based sorting algorithm that uses a binary heap data structure.
  - Time Complexity: O(n log n) in all cases
  - Space Complexity: O(1)

- **Insertion Sort** (`insertion_sort.py`): Builds the final sorted array one item at a time, similar to sorting playing cards in your hand.
  - Time Complexity: O(n²) in worst and average cases, O(n) in best case
  - Space Complexity: O(1)

- **Merge Sort** (`Merge_Sort.py`): A divide-and-conquer algorithm that divides the input array into two halves, recursively sorts them, and then merges the sorted halves.
  - Time Complexity: O(n log n) in all cases
  - Space Complexity: O(n)

- **Quick Sort** (`quick_sort.py`): Another divide-and-conquer algorithm that picks an element as a pivot and partitions the array around the pivot.
  - Time Complexity: O(n log n) in average case, O(n²) in worst case
  - Space Complexity: O(log n) due to recursion stack

- **Selection Sort** (`selection_sort.py`): A simple in-place comparison sort that divides the input into a sorted and an unsorted region and iteratively shrinks the unsorted region.
  - Time Complexity: O(n²) in all cases
  - Space Complexity: O(1)

## Usage

Each sorting algorithm can be run independently. For example:

```bash
python bubbleSort.py
```

Most implementations include a test array that will be sorted and printed when the script is executed.

## Algorithm Selection Guide

- Use **Bubble Sort**, **Insertion Sort**, or **Selection Sort** for small datasets or educational purposes.
- Use **Merge Sort** or **Heap Sort** when you need guaranteed O(n log n) performance.
- Use **Quick Sort** for in-memory sorting of large datasets (but be aware of worst-case behavior).
- Use **Counting Sort** when sorting integers with a limited range.

## Contributing

Feel free to contribute additional sorting algorithms or optimizations to the existing implementations. Please include:

1. Well-commented code explaining the algorithm
2. A test case demonstrating the algorithm in action
3. Time and space complexity analysis

## License

This project is open-source and available under the [MIT License](LICENSE).
