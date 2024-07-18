import { ReactComponentElement } from "react";


declare module './LiveLineCharts' {
  // import { FC } from 'react';

  // interface LiveLineChartsProps {
  //   dataToShowNames: string[];
  //   dataToShowIndexes: number[];
  //   colors: { solidColor: string, transparentColor: string }[];
  // }

  // const LiveLineChart: FC<LiveLineChartProps>;
  // export default LiveLineChart;


  const LiveLineCharts: ReactComponentElement;
  export default LiveLineCharts;

  // import { ReactNode } from 'react';

  // interface OpenNewWindowProps {
  //   innerComponent: ReactNode;
  //   label: string;
  // }

  // const OpenNewWindow: ({ innerComponent, label }: OpenNewWindowProps) => JSX.Element;
  // export { OpenNewWindow };
  import { ReactNode } from 'react';

  interface OpenNewWindowProps {
    innerComponent: ReactNode;
    label: string;
  }

  const OpenNewWindow: ({ innerComponent, label }: OpenNewWindowProps) => JSX.Element;
  export { OpenNewWindow };
}


// declare module './OpenNewWindow' {
//   import { ReactNode } from 'react';

//   interface OpenNewWindowProps {
//     innerComponent: ReactNode;
//     label: string;
//   }

//   const OpenNewWindow: ({ innerComponent, label }: OpenNewWindowProps) => JSX.Element;
//   export { OpenNewWindow };
// }
// declare module './LiveLineChart' {
//   import { ReactNode } from 'react';

//   interface OpenNewWindowProps {
//     innerComponent: ReactNode;
//     label: string;
//   }

//   const OpenNewWindow: ({ innerComponent, label }: OpenNewWindowProps) => JSX.Element;
//   export { OpenNewWindow };
// }

