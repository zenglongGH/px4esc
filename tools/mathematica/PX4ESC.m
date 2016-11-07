(* ::Package:: *)

(*
 * Functions for parsing firmware generated data dumps for plotting.
 * Example usage:
 *
 *  Needs["PX4ESC`", "tools/mathematica/PX4ESC.m"]
 *  lines = PX4ESC`parseFile["file.txt"];
 *  GraphicsColumn[Map[ListLinePlot, PX4ESC`combineXY[lines]]]
 *  {x,data}=PX4ESC`splitXY[lines];
 *  ListLinePlot[{{x,data[[1]]}\[Transpose],{x,data[[2]]}\[Transpose]}]
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

splitXY[lines_] := {lines\[Transpose][[1]], lines\[Transpose][[2 ;;]]};

combineXY[lines_] := Map[{lines\[Transpose][[1]], #}\[Transpose] &, lines\[Transpose][[2 ;;]]];

EndPackage[]
