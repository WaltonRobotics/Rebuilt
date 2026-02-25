package frc.util;

import java.util.*;
import java.util.stream.Stream;

public class BoundedQueue<T> implements Iterable<T> {

    private final int maxSize_;
    private final ArrayList<T> list_;

    public BoundedQueue(int maxSize) {
        maxSize_ = maxSize;
        list_ = new ArrayList<>(maxSize);
    }

    public void add(T item) {
        if (list_.size() >= maxSize_) {
            list_.remove(0);
        }
        list_.add(item);
    }

    public T get(int index) {
        return list_.get(index);
    }

    public T remove(int index) {
        return list_.remove(index);
    }

    public int size() {
        return list_.size();
    }

    public Stream<T> stream() {
        return list_.stream();
    }

    @Override
    public Iterator<T> iterator() {
        return list_.iterator();
    }

    public void clear() {
        list_.clear();
    }
}
