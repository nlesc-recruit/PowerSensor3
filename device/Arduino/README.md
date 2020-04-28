## Notes
The stm32duino library does not create a .bin file in the sketch folder. Which is what the CLI's upload command looks for. This can be solved by either copying the .bin file from its temporary location and renaming it accordingly. Another approach is to convert the .hex or .elf file to a .bin file in the sketch folder and name it accordingly. These options are not very smooth.

As of the 28th of March it should have been fixed, it is not (ref commit/merge?? [6aa5ee9]). The current solution is to remove, or comment, the lines:

	recipe.output.tmp_file={build.project_name}.hex
	recipe.output.save_file={build.project_name}.{build.variant}.hex

These lines are found in:

	$HOME/.arduino15/packages/STM32/hardware/stm32/1.8.0/platform.txt

Please try to compile the sketch first. Then check whether there is a __{projectname}.STM32:stm32:Disco.bin__ file in the sketch folder. If there is, it can be uploaded with out trouble. Otherwise follow the solution described above.
