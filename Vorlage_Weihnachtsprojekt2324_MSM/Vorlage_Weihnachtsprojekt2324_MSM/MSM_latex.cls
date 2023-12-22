\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{MSM_latex}

\newcommand{\headlinecolor}{\normalcolor}
\LoadClass{scrartcl}

\RequirePackage{ngerman}
\RequirePackage[utf8x]{inputenc}
\RequirePackage{amsmath,amssymb,amsthm}
\RequirePackage{graphicx}
\RequirePackage{xcolor}
\RequirePackage{subfigure}
\RequirePackage{bm}
\RequirePackage{float}
\RequirePackage{units}
\RequirePackage{geometry}
\RequirePackage{hyperref}
\RequirePackage{booktabs}
\RequirePackage{hyperref}
\RequirePackage{mathtools}
\RequirePackage{soul}

\setlength{\parindent}{0cm}
\setlength{\parskip}{1em}
\geometry{a4paper,twoside,left=20mm,right=20mm,top=2.5cm,bottom=2.5cm} 

\RequirePackage{selinput}
\usepackage{tikz}
\usetikzlibrary{calc, shapes, backgrounds, bending, decorations.pathmorphing, positioning, decorations.pathreplacing}
\usepgflibrary{arrows}
\RequirePackage{pgfplots}

% Kopf-und Fusszeilen
\usepackage{fancyhdr}
\pagestyle{fancy}
\setlength{\headheight}{30pt}
\setlength{\footskip}{10pt}
% Kopfzeile
\lhead{
	\sffamily
	%        \begin{minipage}[t]{7cm}
	\begin{minipage}[t]{0.9cm}
		\includegraphics[height=0.9cm]{img/itmlogo}
	\end{minipage}
	\begin{minipage}[b]{9cm}
		Institut für Techn.
		und Num. Mechanik\\
		Profs. Eberhard / Hanss / Fehr
	\end{minipage}
}
\chead{}
\rhead{
	\begin{minipage}[b]{8cm}
		\sffamily
		\flushright
		Modell. und Sim. in der Mechatronik\\
		Weihnachtsprojekt (WS 23/24)
		
		%		\textsf{\lesson\\ \prof \semester  A 4.\thepage}
		%             \textsf{ \\ - }
	\end{minipage}
}
\cfoot{}
\rfoot{\flushright \thepage}
\renewcommand{\headrulewidth}{0.4pt}

\hypersetup{pdfinfo={%
  Title={Weihnachtsprojekt WS23/24 -  Modellierung und Simulation in der Mechatronik},
  Creator={Mario Rosenfelder},
  Producer={Mario Rosenfelder},
  Subject={Weihnachtsprojekt},
  Keywords={Weihnachtsprojekt, MSM}
}}

\date{\today}


\definecolor{slcolor}{HTML}{882B21}
