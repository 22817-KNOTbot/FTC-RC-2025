package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;
import java.util.NavigableSet;
import java.util.TreeSet;

import com.qualcomm.robotcore.util.Range;

public class InterpolatedLUT<T extends Number> {
	public class Entry {
		public double input;
		public T output;

		public Entry(double input, T output) {
			this.input = input;
			this.output = output;
		}
	}

	private NavigableSet<Entry> entriesSet;

	public InterpolatedLUT() {
		entriesSet = new TreeSet<>((entry1, entry2) -> Double.compare(entry1.input, entry2.input));
	}

	@SafeVarargs
	public final void add(Entry... entries) {
		for (Entry entry : entries) {
			entriesSet.add(entry);
		}
	}

	public T get(double input) {
		Entry compareEntry = new Entry(input, null);
		Entry lowerEntry = entriesSet.floor(compareEntry);
		Entry higherEntry = entriesSet.ceiling(compareEntry);

		if (lowerEntry == null)
			lowerEntry = higherEntry;
		if (higherEntry == null)
			higherEntry = lowerEntry;

		if (lowerEntry == null && higherEntry == null) {
			return null;
		}

		Double scaledOutput = Range.scale(input, lowerEntry.input, higherEntry.input, 
				lowerEntry.output.doubleValue(), higherEntry.output.doubleValue());
		
		try {
			@SuppressWarnings("unchecked")
			T output = (T) scaledOutput;
			return output;
		} catch (ClassCastException e) {
			return null;
		}
	}
}
