def insertion_sort(arr):
    for i in range(1, len(arr)):
        j = i
        while j > 0 and arr[j] < arr[j-1]:
            arr[j], arr[j-1] = arr[j-1], arr[j]
            j -= 1
    return arr
print(insertion_sort([2, 1, 6, 3, 5, 4, 7, 8, 9, 0]))  # [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
