import csv
import os
import sys

def read_csv(file_path):
    try:
        with open(file_path, mode='r') as csv_file:
            csv_reader = csv.reader(csv_file)
            headers = next(csv_reader)  # Read the header row
            print(f"Headers: {headers}")
            
            print("First few rows of data:")
            for i, row in enumerate(csv_reader):
                print(row)
                if i >= 9:  # Display only the first 10 rows
                    break

    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    csv_file = sys.argv[1]
    if os.path.exists(csv_file):
        read_csv(csv_file)