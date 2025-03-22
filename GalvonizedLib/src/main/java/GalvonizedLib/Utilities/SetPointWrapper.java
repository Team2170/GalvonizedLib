package GalvonizedLib.Utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A wrapper class for managing an immutable, sorted list of set points (doubles) and providing
 * utility methods.
 */
public final class SetPointWrapper {
  private final List<Double> setPoints;

  /**
   * Constructs a SetPointWrapper from a comma-separated string of points. The points are parsed,
   * sorted, and stored in an immutable list.
   *
   * @param points a comma-separated string of double values.
   * @throws NumberFormatException if any value in the string cannot be parsed as a double.
   */
  public SetPointWrapper(String points) {
    this.setPoints =
        Collections.unmodifiableList(
            Arrays.stream(points.split(","))
                .map(Double::parseDouble)
                .sorted()
                .collect(Collectors.toList()));
  }

  /**
   * Constructs a SetPointWrapper from a list of points. The list is copied, sorted, and stored in
   * an immutable list.
   *
   * @param points a list of double values.
   */
  public SetPointWrapper(List<Double> points) {
    this.setPoints =
        Collections.unmodifiableList(points.stream().sorted().collect(Collectors.toList()));
  }

  /**
   * Retrieves the set point at the specified index.
   *
   * @param index the index of the set point.
   * @return the set point at the specified index.
   * @throws IndexOutOfBoundsException if the index is out of bounds.
   */
  public double get(int index) {
    return setPoints.get(index);
  }

  /**
   * Retrieves the next set point relative to the given index. If the index is the last one, it
   * returns the same set point.
   *
   * @param index the current index.
   * @return the next set point or the current set point if at the end of the list.
   */
  public double next(int index) {
    int max = setPoints.size() - 1;
    if (index == max) {
      return setPoints.get(max);
    }
    return setPoints.get(index + 1);
  }

  /**
   * Retrieves the previous set point relative to the given index. If the index is the first one, it
   * returns the same set point.
   *
   * @param index the current index.
   * @return the previous set point or the current set point if at the beginning of the list.
   */
  public double previous(int index) {
    if (index == 0) {
      return setPoints.get(0);
    }
    return setPoints.get(index - 1);
  }

  /**
   * Finds the nearest set point that is greater than or equal to the given value. If no such set
   * point exists, returns the last set point.
   *
   * @param current the value to compare against the set points.
   * @return the nearest set point that is greater than or equal to the given value.
   */
  public double findNearestSetPoint(double current) {
    if (setPoints == null || setPoints.isEmpty()) {
      throw new IllegalArgumentException("The list cannot be null or empty.");
    }

    Double nearest = null;
    double smallestDifference = Double.MAX_VALUE;

    for (Double number : setPoints) {
      double difference = Math.abs(number - current);
      if (difference < smallestDifference) {
        smallestDifference = difference;
        nearest = number;
      }
    }
    return nearest;
  }

  /**
   * Returns an immutable copy of the set points.
   *
   * @return an unmodifiable list of the set points.
   */
  public List<Double> getPoints() {
    return setPoints;
  }

  public List<Double> getSurroundingPoints(Rotation2d pos) {
    double currentClosestPoint = findNearestSetPoint(pos.getRotations());

    int index = getIndexFromValue(currentClosestPoint);
    double prevPoint = previous(index);
    double nextPoint = next(index);
    List<Double> surroundingPoints = new ArrayList<Double>() {};
    surroundingPoints.add(prevPoint);
    surroundingPoints.add(nextPoint);

    return surroundingPoints;
  }

  /**
   * Finds the index of a specified value in the list of set points.
   *
   * @param value The value to locate in the set points list.
   * @return The index of the value in the list, or -1 if the value is not found.
   */
  public int getIndexFromValue(double value) {
    return setPoints.indexOf(value);
  }
}
