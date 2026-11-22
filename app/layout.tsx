import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  metadataBase: new URL("https://embodied-path-planning-lab.promisingcypress10.chatgpt.site"),
  title: "NAV STUDIO | 路径规划工作台",
  description: "全屏交互式 A*、Dijkstra、RRT 与 RRT* 路径规划工作台。",
  openGraph: {
    title: "NAV STUDIO | 路径规划工作台",
    description: "在一个全屏工作台中观察 A*、Dijkstra、RRT 与 RRT* 的搜索、障碍膨胀和路径生成。",
    images: [{ url: "/og.png", width: 1733, height: 907, alt: "NAV STUDIO 路径规划工作台" }],
  },
  twitter: {
    card: "summary_large_image",
    title: "NAV STUDIO | 路径规划工作台",
    description: "全屏交互式路径规划算法工作台。",
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
