# RoboLab Presentation 08 - Jupyter Notebooks

## Visual Studio Code / Jupyter Notebook Cheat-Sheet

| Button                | Meaning                                                                 |
| --------------------- | ----------------------------------------------------------------------- |
| `Ctrl+S`              | Save                                                                    |
| `ESC`                 | Change the cell mode                                                    |
| `A`                   | Add a cell above                                                        |
| `B`                   | Add a cell below                                                        |
| `J` or down arrow key | Change a cell to below                                                  |
| `K` or up arrow key   | Change a cell to above                                                  |
| `Ctrl`+`Enter`        | Run the currently selected cell                                         |
| `Shift`+`Enter`       | Run the currently selected cell and insert a new cell immediately below |
| `dd`                  | Delete a selected cell                                                  |
| `z`                   | Undo the last change                                                    |
| `M`                   | switch the cell type to Markdown                                        |
| `Y`                   | switch the cell type to code                                            |
| `L`                   | Enable/Disable line numbers                                             |

Take a look at the materials of the course:

+ all theoretical and practical materials [https://github.com/SebastianZug/PythonCourse_2023](https://github.com/SebastianZug/PythonCourse_2023)
+ Interactive LiaScript document - [bit.ly/3r1sRug](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/PythonCourse_2023/main/README.md#1)
+ this document [https://github.com/SebastianZug/PythonCourse_2023/blob/main/cheat_sheet.md](https://github.com/SebastianZug/PythonCourse_2023/blob/main/cheat_sheet.md)


This pdf was generate by pandoc from the markdown file. The command used was:

```bash
pandoc cheat_sheet.md -V geometry:landscape -V fontsize=11pt 
-V link-citations:true -V colorlinks:true -f markdown -o cheat_sheet.pdf
```