using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Reflection;

namespace x_IMU_Binary_File_Converter
{
    class Program
    {
        /// <summary>
        /// x-IMU Binary file converter.
        /// </summary>
        /// <param name="args">
        /// -src: CurrentDirectory. Source file or directory path of binary file to be converted.  If directory, all *.bin files will be converted.
        /// -sub: False. Operate on all subdirectory.  Only applies if source path is a directory.
        /// -ext: False. Automatically exit application on successful completion.  Otherwise user must "Press any key to exit."
        /// </param>
        /// <returns>
        /// False if completed successfully. True if an error occurred, error details printed in console.
        /// </returns>
        static int Main(string[] args)
        {
            Console.WriteLine(Assembly.GetExecutingAssembly().GetName().Name + " " + Assembly.GetExecutingAssembly().GetName().Version.Major.ToString() + "." + Assembly.GetExecutingAssembly().GetName().Version.Minor.ToString());
            try
            {
                #region Variables

                string src = Directory.GetCurrentDirectory();
                bool sub = false;
                bool ext = false;
                string[] binaryFilePaths = null;

                #endregion

                #region Collect arguments

                if (args.Length % 2 != 0)
                {
                    throw new Exception("Arguments must be in pairs.");
                }
                for (int i = 0; i < args.Length; i += 2)
                {
                    switch (args[i])
                    {
                        case ("-src"): src = args[i + 1]; break;
                        case ("-ext"): ext = Convert.ToBoolean(args[i + 1]); break;
                        case ("-sub"): sub = Convert.ToBoolean(args[i + 1]); break;
                        default:
                            throw new Exception("Invalid argument.");
                    }
                }

                Console.WriteLine("-src: " + src);
                Console.WriteLine("-sub: " + Convert.ToString(sub));
                Console.WriteLine("-ext: " + Convert.ToString(ext));

                #endregion

                #region Create array of file paths

                if (File.Exists(src))
                {
                    binaryFilePaths = new string[] { src };
                }
                else if (File.Exists(src + ".bin"))
                {
                    binaryFilePaths = new string[] { src };
                }
                else if (Directory.Exists(src))
                {
                    binaryFilePaths = Directory.GetFiles(src, "*.bin", sub ? SearchOption.AllDirectories : SearchOption.TopDirectoryOnly);
                }
                else
                {
                    throw new Exception("Source file or directory does not exist.");
                }

                if (binaryFilePaths.Length == 1)
                {
                    Console.WriteLine("Found " + binaryFilePaths[0] + ".");
                }
                else if (binaryFilePaths.Length > 1)
                {
                    Console.WriteLine("Found " + Convert.ToString(binaryFilePaths.Length) + " *.bin files.");
                }
                else
                {
                    throw new Exception("No *.bin files found in directory.");
                }

                #endregion

                #region Convert each file

                for (int i = 0; i < binaryFilePaths.Length; i++)
                {
                    Console.Write("Converting " + Path.GetFileName(binaryFilePaths[i]) + "...");
                    x_IMU_API.CSVfileWriter CSVfileWriter = new x_IMU_API.CSVfileWriter(Path.GetDirectoryName(binaryFilePaths[i]) + "\\" + Path.GetFileNameWithoutExtension(binaryFilePaths[i]));
                    x_IMU_API.xIMUfile xIMUfile = new x_IMU_API.xIMUfile(binaryFilePaths[i]);
                    xIMUfile.xIMUdataRead += new x_IMU_API.xIMUfile.onxIMUdataRead(delegate(object sender, x_IMU_API.xIMUdata e) { CSVfileWriter.WriteData(e); });
                    xIMUfile.Read();
                    xIMUfile.Close();
                    CSVfileWriter.CloseFiles();
                    Console.Write("Done. Packets read: " + Convert.ToString(xIMUfile.PacketsReadCounter.TotalPackets) + ", Read errors: " + Convert.ToString(xIMUfile.ReadErrors) + "\n");
                }

                #endregion

                Console.WriteLine("Complete.");
                if (ext)
                {
                    return 0;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error: " + ex.Message);
                Console.WriteLine("Press any key to exit...");
                Console.ReadKey();
                return 1;
            }
            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();
            return 0;
        }
    }
}