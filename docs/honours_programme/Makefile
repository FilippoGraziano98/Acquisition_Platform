SRC = main.tex

OUT = honours_programme_report#relazione_graziano_filippo_1761694

.phony: clean all

all:
	pdflatex -shell-escape -jobname=$(OUT) $(SRC)
	biber $(OUT).bcf
	pdflatex -shell-escape -jobname=$(OUT) $(SRC)
	pdflatex -shell-escape -jobname=$(OUT) $(SRC)
	


clean:
	rm -rf *~ *.toc *.bcf *.aux *.run.xml *.out *.log *.bbl *.blg
