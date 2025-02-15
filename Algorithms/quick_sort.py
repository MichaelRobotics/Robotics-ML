# quadratic running time O(n^2)
def quicksort(arr, left, right):
    if left < right:
        partition_index = partition(arr, left, right)
        quicksort(arr, left, partition_index - 1)
        quicksort(arr, partition_index + 1, right)

def partition(arr, left, right):
    i = left
    j = right - 1
    pivot = arr[right]
    while i < j:
        while arr[i] < pivot and i < right:
            i += 1
        while arr[j] > pivot and j > left:
            j -= 1
        if i < j:
            arr[i], arr[j] = arr[j], arr[i]
    if arr[i] > pivot:
        arr[i], arr[right] = arr[right], arr[i]

    return i

arr = [2, 1, 6, 3, 5, 4, 7, 8, 9, 0]
quicksort(arr, 0, len(arr)-1)
print(arr)  # [1, 2, 3, 4, 5]
