package org.firstinspires.ftc.teamcode.util;

import java.util.PriorityQueue;

public class RollingTimeQueue<E> {
	private PriorityQueue<TimedElement> queue;
	private long timeLimitMs;

	private class TimedElement implements Comparable<TimedElement> {
		E element;
		long timestamp;

		TimedElement(E element, long timestamp) {
			this.element = element;
			this.timestamp = timestamp;
		}

		@Override
		public int compareTo(TimedElement other) {
			return Long.compare(this.timestamp, other.timestamp);
		}
	}

	public RollingTimeQueue(long timeLimitMs) {
		this.timeLimitMs = timeLimitMs;
		this.queue = new PriorityQueue<>();
	}

	public void add(E element) {
		long currentTime = System.currentTimeMillis();
		queue.add(new TimedElement(element, currentTime));
		cleanup(currentTime);
	}

	public E peek() {
		cleanup(System.currentTimeMillis());
		if (!queue.isEmpty()) {
			return queue.peek().element;
		}
		return null;
	}

	public E poll() {
		cleanup(System.currentTimeMillis());
		if (!queue.isEmpty()) {
			return queue.poll().element;
		}
		return null;
	}

	public void clear() {
		queue.clear();
	}

	private void cleanup(long currentTime) {
		while (!queue.isEmpty() && (currentTime - queue.peek().timestamp) > timeLimitMs) {
			queue.poll();
		}
	}
}
