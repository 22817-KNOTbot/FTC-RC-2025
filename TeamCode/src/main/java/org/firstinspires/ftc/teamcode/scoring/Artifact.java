package org.firstinspires.ftc.teamcode.scoring;

public class Artifact {
	public enum Colour {
		PURPLE,
		GREEN
	}

	public enum Pattern {
		GPP(Colour.GREEN, Colour.PURPLE, Colour.PURPLE),
		PGP(Colour.PURPLE, Colour.GREEN, Colour.PURPLE),
		PPG(Colour.PURPLE, Colour.PURPLE, Colour.GREEN);

		private Colour[] pattern;

		private Pattern(Colour colour1, Colour colour2, Colour colour3) {
			pattern = new Colour[] { colour1, colour2, colour3 };
		}

		public Colour[] getPattern() {
			return pattern;
		}
	}
}
