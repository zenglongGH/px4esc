(*
 * Functions for parsing firmware generated data dumps for plotting.
 * Example usage:
 *
 *  Needs["PX4ESC`", "tools/mathematica/PX4ESC.m"]
 *  plots = PX4ESC`splitXY[PX4ESC`parseFile["file.txt"]];
 *  GraphicsColumn[Map[ListLinePlot, plots]]
 *)

BeginPackage["PX4ESC`"]

parseFile[file_] := Module[{
   strings = Select[Import[file, "Lines"], StringLength[#1] > 0 && Characters[#1][[1]] == "$" &],
   lines,
   varsPerLine
  },
  lines = Map[Map[ToExpression, StringSplit[StringDrop[#, 1], ","]] &, strings];
  varsPerLine = Median[Map[Length, lines]];
  Select[lines, Length[#] == varsPerLine &]
];

splitXY[lines_] := Map[{lines\[Transpose][[1]], #}\[Transpose] &, lines\[Transpose][[2 ;;]]];

EndPackage[]
