package frc.robot;

import java.util.TreeMap;

public class PoseSetter {

	public static final String Feeder = "Feeder";
	public static final String Zero = "Zero";
	public static final String Stored = "Stored";
	public static final String Ground = "Ground";
	// public static final String Climb = "Climb";
	// public static final String PreClimb = "PreClimb";
	public static final String PreZero = "PreZero";
	public static final String AlgaeGrab = "AlgaeRemove";
	public static final String AlgaePlace = "AlgaePlace";
	public static final String StartPreMatch = "StartPreMatch";

	public static final TreeMap<String, double[]> positionsMap = new TreeMap<String, double[]>() {
		private static final long serialVersionUID = 1L;
		{ // place to go to stage 1, stage 2, shoulder

			put("L1", new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2LowerLimit,
					103
			}); 

			put("L2", new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2LowerLimit,
					103,
			});

			put("L3", new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					2.66,
					106,
			}); 

			put("L4", new double[] {
					Constants.ElevatorConstants.stage1UpperLimit + 0.38,
					Constants.ElevatorConstants.stage2UpperLimit + 0.29,
					116,
			});

			put(Feeder, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					1.09,
					Constants.ShoulderConstants.shoulderUpperLimit,
			});

			put(Zero, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2UpperLimit,
					Constants.ShoulderConstants.shoulderLowerLimit,
			});

			put(StartPreMatch, new double[] {
				-0.6,
				1.66,
				Constants.ShoulderConstants.shoulderLowerLimit,
			});

			put(Stored, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2LowerLimit + 0.2,
	 				120,
			});

			// put(Climb, new double[] {
			// 		-2.26,
			// 		1.39,
			// 		106.6,
			// });

			// put(PreClimb, new double[] {
			// 		1.4,
			// 		Constants.ElevatorConstants.stage2UpperLimit,
			// 		212.5,
			// });

			put(PreZero, new double[] {
					Constants.ElevatorConstants.stage1UpperLimit / 4,
					Constants.ElevatorConstants.stage2UpperLimit - 0.8,
					Constants.ShoulderConstants.shoulderLowerLimit + 35,
			});

			// ground
			put(AlgaeGrab + "L1", new double[] {
				Constants.ElevatorConstants.stage1LowerLimit,
				Constants.ElevatorConstants.stage2LowerLimit,
				200,
			}); 

			put(AlgaeGrab + "L2", new double[] {
				Constants.ElevatorConstants.stage1LowerLimit,
				Constants.ElevatorConstants.stage2UpperLimit,
				29.4,
			});

			put(AlgaeGrab + "L3", new double[] {
				4.6,
				Constants.ElevatorConstants.stage2UpperLimit,
				29.6,
		    	});

			put(AlgaeGrab + "L4", new double[] {
				4.6,
				Constants.ElevatorConstants.stage2UpperLimit,
				29.4,
		    	});

			// processor
			put(AlgaePlace + "L1", new double[]{
				Constants.ElevatorConstants.stage1LowerLimit - 0.4,
				Constants.ElevatorConstants.stage2LowerLimit,
				206.56,
			}); 

			// processor
			put(AlgaePlace + "L2", new double[]{
				Constants.ElevatorConstants.stage1LowerLimit - 0.4,
				Constants.ElevatorConstants.stage2LowerLimit,
				206.56,
			}); 

			// barge
			put(AlgaePlace + "L3", new double[] {
				Constants.ElevatorConstants.stage1UpperLimit + 0.36,
				Constants.ElevatorConstants.stage2UpperLimit + 0.29,
				127
		    	}); 

			// barge
			put(AlgaePlace + "L4", new double[] {
				Constants.ElevatorConstants.stage1UpperLimit + 0.36,
				Constants.ElevatorConstants.stage2UpperLimit + 0.29,
				127
		    	}); 
		}
	};

}
