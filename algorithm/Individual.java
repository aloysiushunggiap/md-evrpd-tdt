package algorithm;

import model.Solution;

import java.util.List;

public class Individual {
    public List<Integer> chromosome;
    public Solution solution;
    public double fitness;   // chỉ dùng nội bộ cho ICGA, không phải cost in ra

    public Individual(List<Integer> chromosome) {
        this.chromosome = chromosome;
        this.fitness = Double.MAX_VALUE;
    }
}