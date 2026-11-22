import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  metadataBase: new URL("https://embodied-path-planning-lab.promisingcypress10.chatgpt.site"),
  title: "Path Lab 06 | 具身智能路径规划演示",
  description: "交互式演示 A*、Dijkstra 与 RRT 的搜索、障碍膨胀和代价地图。",
  openGraph: {
    title: "PATH LAB 06 | 具身智能路径规划演示",
    description: "看见 A*、Dijkstra 与 RRT 的 GENERATE、PICK、障碍膨胀和代价传播。",
    images: [{ url: "/og.png", width: 1734, height: 907, alt: "PATH LAB 06 具身智能路径规划演示" }],
  },
  twitter: {
    card: "summary_large_image",
    title: "PATH LAB 06 | 具身智能路径规划演示",
    description: "交互式路径规划算法可视化实验室。",
    images: ["/og.png"],
  },
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="zh-CN">
      <body>{children}</body>
    </html>
  );
}
