package frc.robot.utils.math.units;

import java.util.ArrayList;

public class UnitBuilder {
    private BaseUnit[] nums = new BaseUnit[] {};
    private BaseUnit[] denoms = new BaseUnit[] {};

    public UnitBuilder num(BaseUnit... nums) {
        this.nums = nums;

        return this;
    }

    public UnitBuilder denom(BaseUnit... denoms) {
        this.denoms = denoms;

        return this;
    }

    public CompositeUnit make() {
        return new CompositeUnit(nums, denoms);
    }
}